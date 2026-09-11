def matching_elements(list1, list2):
    """Values common to both lists - unlike count_matching_elements, callers
    that need to inspect *which* values matched (not just how many) use this
    directly."""
    return set(list1) & set(list2)


def count_matching_elements(list1, list2):
    return len(matching_elements(list1, list2))


def is_valid_percentage(values):
    """True if `values` is non-empty and every item parses as a number in [0, 100]."""
    if not values:
        return False
    try:
        return all(0.0 <= float(v) <= 100.0 for v in values)
    except (TypeError, ValueError):
        return False


def any_high_risk_percentage(values):
    """True if at least one item in `values` parses as a number in the
    high-risk band (66-100) shared by every sensor's risk-percentage config
    (see e.g. sensor/config/oximeter.yaml's highrisk_percent). Checks for
    *any* match rather than requiring every value to qualify, unlike
    is_valid_percentage - callers capture a window of readings where only
    some are expected to land in the high-risk band.
    """
    try:
        return any(66.0 <= float(v) <= 100.0 for v in values)
    except (TypeError, ValueError):
        return False
