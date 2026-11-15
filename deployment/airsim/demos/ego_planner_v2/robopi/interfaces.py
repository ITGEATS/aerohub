"""Explicit interface mapping shared by runtime, clients and evidence collection."""


def probe_prefix(run):
    from policy import run_id
    return '/acceptance/run_'+run_id(run).replace('-', '_')+'/'


def topic_map(config):
    if not config.get('vehicle'):
        return {}
    if config['vehicle'] not in ('drone0', 'drone1'):
        raise ValueError('Supported compute scopes are drone0 and drone1')
    ns = config['vehicle']
    return {
        '/ego/goal': f'/{ns}/ego/goal',
        '/ego/cancel': f'/{ns}/ego/cancel',
        '/ego/status': f'/{ns}/ego/status',
        '/ego/drone0/odom': f'/{ns}/ego/native/odom',
        '/ego/drone0/points': f'/{ns}/ego/native/points',
        '/planner/trajectory': f'/{ns}/planner/trajectory',
        '/planner/failure': f'/{ns}/planner/failure',
    }
