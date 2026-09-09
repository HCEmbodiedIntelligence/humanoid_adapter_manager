"""Device instance values and plugin-owned schemas; no vendor class knowledge."""
import copy
import math
import re

from .deployment import DeploymentError

SETTINGS_DEFAULTS = {'instance_parameters': {}, 'startup': [], 'capabilities': {}}
REFERENCE = re.compile(r'\$\{([A-Za-z_][A-Za-z0-9_]*)\}')


def settings_from_manifest(manifest):
    return {key: copy.deepcopy(manifest.get(key, default)) for key, default in SETTINGS_DEFAULTS.items()}


def validate_settings(settings):
    if not isinstance(settings, dict) or set(settings) - SETTINGS_DEFAULTS.keys():
        raise DeploymentError('device settings contain unknown fields')
    result = {**copy.deepcopy(SETTINGS_DEFAULTS), **copy.deepcopy(settings)}
    values = result['instance_parameters']
    if not isinstance(values, dict) or len(values) > 128:
        raise DeploymentError('instance_parameters must be a mapping of at most 128 strings')
    for name, value in values.items():
        if (not isinstance(name, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name)
                or not isinstance(value, str) or len(value) > 4096 or '\x00' in value
                or '$(' in value or '${' in value):
            raise DeploymentError('instance_parameters require literal string values and valid names')
    from .plugin_startup import validate_startup
    validate_startup(expand(result['startup'], values))
    capabilities = result['capabilities']
    if not isinstance(capabilities, dict):
        raise DeploymentError('plugin capabilities must be a mapping')
    grippers = capabilities.get('grippers', {})
    if not isinstance(grippers, dict):
        raise DeploymentError('capabilities.grippers must be a mapping')
    for name, targets in grippers.items():
        if not isinstance(name, str) or not isinstance(targets, dict):
            raise DeploymentError('gripper capabilities require named target mappings')
        for key in ('open_position', 'closed_position', 'max_effort'):
            if key in targets and (isinstance(targets[key], bool) or not isinstance(targets[key], (int, float))
                                   or not math.isfinite(targets[key])):
                raise DeploymentError(f'{name}.{key} must be finite')
        if targets.get('max_effort', 0) < 0:
            raise DeploymentError(f'{name}.max_effort must be nonnegative')
    return result


def expand(value, parameters):
    if isinstance(value, str):
        def substitute(match):
            name = match.group(1)
            if name not in parameters:
                raise DeploymentError(f'undefined device instance parameter: {name}')
            return parameters[name]
        return REFERENCE.sub(substitute, value)
    if isinstance(value, dict):
        result = {}
        for key, item in value.items():
            resolved_key = expand(key, parameters) if isinstance(key, str) else key
            if resolved_key in result:
                raise DeploymentError(f'device instance expansion creates duplicate key: {resolved_key}')
            result[resolved_key] = expand(item, parameters)
        return result
    if isinstance(value, list):
        return [expand(item, parameters) for item in value]
    return copy.deepcopy(value)


def resolved_document(document, manifest):
    return expand(document, manifest.get('instance_parameters', {}))


def _parameter_schema_validator(schema):
    import jsonschema

    # Preserve the original 2020-12 contract for schemas without a declaration.
    # Explicit draft-07 schemas also work with Ubuntu 22.04's jsonschema 3.2.
    dialect = 'https://json-schema.org/draft/2020-12/schema'
    if isinstance(schema, dict):
        dialect = schema.get('$schema', dialect)
    supported = {
        'http://json-schema.org/draft-07/schema': ('Draft7Validator', '3'),
        'https://json-schema.org/draft-07/schema': ('Draft7Validator', '3'),
        'https://json-schema.org/draft/2020-12/schema': ('Draft202012Validator', '4'),
    }
    if not isinstance(dialect, str) or dialect.rstrip('#') not in supported:
        raise DeploymentError('unsupported plugin parameter schema dialect; use draft-07 or 2020-12')
    name, minimum = supported[dialect.rstrip('#')]
    validator = getattr(jsonschema, name, None)
    if validator is None:
        raise DeploymentError(
            f'plugin parameter schema {dialect} requires jsonschema>={minimum} in the Python environment '
            'used for packaging and runtime; draft-07 is supported by Ubuntu 22.04 system Python')
    return validator


def validate_parameter_schema(manifest, parameters):
    schema = resolved_document(manifest.get('parameter_schema'), manifest)
    if schema is None:
        return  # Older plugins retain the common contract and their runtime validation.
    def local_references(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if key in {'$ref', '$dynamicRef'} and (not isinstance(item, str) or not item.startswith('#')):
                    raise DeploymentError('plugin parameter schema must use local references')
                local_references(item)
        elif isinstance(value, list):
            for item in value:
                local_references(item)
    local_references(schema)
    from jsonschema import SchemaError, ValidationError
    validator = _parameter_schema_validator(schema)
    try:
        validator.check_schema(schema)
        converted = dict(parameters)
        properties = schema.get('properties', {}) if isinstance(schema, dict) else {}
        for name, value in parameters.items():
            rule = properties.get(name, {})
            kind = rule.get('type') if isinstance(rule, dict) else None
            try:
                if kind == 'number': converted[name] = float(value)
                elif kind == 'integer': converted[name] = int(value)
                elif kind == 'boolean' and value in {'true', 'false'}: converted[name] = value == 'true'
            except ValueError:
                pass  # The schema reports the type mismatch with the parameter name.
            if isinstance(converted[name], float) and not math.isfinite(converted[name]):
                raise DeploymentError(f'plugin parameter {name} must be finite')
        validator(schema).validate(converted)
    except (SchemaError, ValidationError) as error:
        raise DeploymentError(f'plugin parameter schema: {error.message}') from error
