"""Personalised task: P = (last two digits of student ID) mod 6."""

# P -> (colour in Zone A, colour in Zone B, colour in Zone C)
_TABLE = {
    0: ('red', 'yellow', 'blue'),
    1: ('red', 'blue', 'yellow'),
    2: ('yellow', 'red', 'blue'),
    3: ('yellow', 'blue', 'red'),
    4: ('blue', 'red', 'yellow'),
    5: ('blue', 'yellow', 'red'),
}


def compute_p(student_id) -> int:
    sid = str(student_id).strip()
    if len(sid) < 2 or not sid.isdigit():
        raise ValueError(f'student_id must contain at least 2 digits, got {student_id!r}')
    return int(sid[-2:]) % 6


def zone_assignment(student_id) -> dict:
    """Return {'zone_a': 'blue_cube', 'zone_b': ..., 'zone_c': ...}."""
    a, b, c = _TABLE[compute_p(student_id)]
    return {'zone_a': f'{a}_cube', 'zone_b': f'{b}_cube', 'zone_c': f'{c}_cube'}


def describe(student_name, student_id) -> str:
    p = compute_p(student_id)
    xx = str(student_id).strip()[-2:]
    assign = zone_assignment(student_id)
    lines = [f'Student: {student_name} ({student_id})',
             f'P = {xx} mod 6 = {p}']
    lines += [f'  {z.replace("zone_", "Zone ").title()} -> {o}' for z, o in assign.items()]
    return '\n'.join(lines)
