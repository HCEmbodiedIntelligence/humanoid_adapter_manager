"""Keep system-Python packaging compatible without weakening newer schema rules."""
import importlib.util
from pathlib import Path

import jsonschema
import pytest
import yaml

from humanoid_manager.deployment import DeploymentError
from humanoid_manager.plugin_metadata import validate_parameter_schema


DRAFT7 = 'http://json-schema.org/draft-07/schema#'
DRAFT2020 = 'https://json-schema.org/draft/2020-12/schema'


def draft7_schema():
    return {'$schema': DRAFT7, 'type': 'object',
            'properties': {'timeout': {'type': 'number', 'exclusiveMinimum': 0},
                           'active': {'type': 'boolean'}},
            'required': ['timeout', 'active'], 'additionalProperties': False}


def test_draft7_does_not_require_2020_validator(monkeypatch):
    monkeypatch.setattr(jsonschema, 'Draft202012Validator', None, raising=False)
    validate_parameter_schema({'parameter_schema': draft7_schema()},
                              {'timeout': '0.5', 'active': 'false'})


@pytest.mark.parametrize('values', [
    {'timeout': '0', 'active': 'false'},
    {'timeout': '-1', 'active': 'false'},
    {'timeout': 'invalid', 'active': 'false'},
    {'timeout': '0.5', 'active': 'invalid'},
    {'timeout': '0.5'},
    {'timeout': '0.5', 'active': 'false', 'extra': 'value'},
])
def test_draft7_still_rejects_invalid_device_parameters(values):
    with pytest.raises(DeploymentError, match='plugin parameter schema'):
        validate_parameter_schema({'parameter_schema': draft7_schema()}, values)


def test_draft7_rejects_invalid_schema():
    schema = draft7_schema()
    schema['properties']['timeout']['type'] = 'unknown_type'
    with pytest.raises(DeploymentError, match='plugin parameter schema'):
        validate_parameter_schema({'parameter_schema': schema}, {})


@pytest.mark.parametrize('declaration', [{}, {'$schema': DRAFT2020}])
def test_missing_2020_validator_has_actionable_error_without_downgrade(monkeypatch, declaration):
    monkeypatch.setattr(jsonschema, 'Draft202012Validator', None, raising=False)
    schema = {**declaration, 'dependentRequired': {'port': ['baud']}}
    with pytest.raises(DeploymentError, match=r'requires jsonschema>=4'):
        validate_parameter_schema({'parameter_schema': schema}, {'port': '/dev/device'})


@pytest.mark.skipif(not hasattr(jsonschema, 'Draft202012Validator'), reason='jsonschema 4 is required for 2020-12')
@pytest.mark.parametrize('declaration', [{}, {'$schema': DRAFT2020}])
def test_2020_keyword_semantics_are_preserved(declaration):
    schema = {**declaration, 'dependentRequired': {'port': ['baud']}}
    with pytest.raises(DeploymentError, match='plugin parameter schema'):
        validate_parameter_schema({'parameter_schema': schema}, {'port': '/dev/device'})
    validate_parameter_schema({'parameter_schema': schema}, {'port': '/dev/device', 'baud': '115200'})


@pytest.mark.parametrize('dialect', ['https://example.invalid/schema', '', None, True])
def test_unknown_dialect_cannot_silently_select_another_validator(dialect):
    with pytest.raises(DeploymentError, match='unsupported plugin parameter schema dialect'):
        validate_parameter_schema({'parameter_schema': {'$schema': dialect}}, {})


@pytest.mark.parametrize('profile', ['ros_topic_gripper.yaml', 'openarmx_v10_bimanual.yaml'])
def test_gripper_packager_emits_system_compatible_schema(profile):
    source = Path(__file__).resolve().parents[2] / 'humanoid_gripper'
    spec = importlib.util.spec_from_file_location('gripper_schema_packager', source / 'tools/create_deployment_bundle.py')
    packager = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(packager)
    config = yaml.safe_load((source / 'config' / profile).read_text())
    manifest = packager.plugin_metadata(config)
    assert manifest['parameter_schema']['$schema'] == DRAFT7
    parameters = dict(entry.split('=', 1) for entry in config['humanoid_gripper_runtime']['ros__parameters']['plugin_parameters'])
    validate_parameter_schema(manifest, parameters)
    parameters['feedback_timeout_s'] = '-1'
    with pytest.raises(DeploymentError, match='plugin parameter schema'):
        validate_parameter_schema(manifest, parameters)
