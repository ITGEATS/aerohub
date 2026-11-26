"""Independent fault verdicts; ordinary navigation failure is not a fault pass."""


def cancel_case_passed(result):
    required = ('fault_injected', 'cancel_ack', 'expected_cancel', 'stopped',
                'land_success', 'disarmed', 'offboard_released')
    revision = result.get('max_effective_revision')
    return (all(result.get(key) is True for key in required)
            and type(revision) is int and revision >= 2
            and result.get('collision') is False and not result.get('error'))
