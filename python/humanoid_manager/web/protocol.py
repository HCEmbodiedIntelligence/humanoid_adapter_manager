import time


def envelope(kind, source, payload, **metadata):
    return {'version':1,'kind':kind,'source':source,'timestamp':time.time(),
            'payload':payload,**metadata}
