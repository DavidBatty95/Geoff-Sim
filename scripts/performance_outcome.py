"""Shared fail-closed outcome contract for native performance evidence."""


def strict_success(result):
    return (isinstance(result, dict) and
            result.get('success') is True and
            result.get('mission_complete') is True and
            result.get('data_valid') is True and
            result.get('fallen') is False and
            type(result.get('contact_samples')) is int and
            result['contact_samples'] == 0)
