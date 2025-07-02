import math
import numpy as np

def unit_vector(data, axis=None, out=None):
    """
    Returns ndarray normalized by length, i.e. eucledian norm, along axis.

    E.g.:
        >>> v0 = numpy.random.random(3)
        >>> v1 = unit_vector(v0)
        >>> numpy.allclose(v1, v0 / numpy.linalg.norm(v0))
        True

        >>> v0 = numpy.random.rand(5, 4, 3)
        >>> v1 = unit_vector(v0, axis=-1)
        >>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=2)), 2)
        >>> numpy.allclose(v1, v2)
        True

        >>> v1 = unit_vector(v0, axis=1)
        >>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=1)), 1)
        >>> numpy.allclose(v1, v2)
        True

        >>> v1 = numpy.empty((5, 4, 3), dtype=numpy.float32)
        >>> unit_vector(v0, axis=1, out=v1)
        >>> numpy.allclose(v1, v2)
        True

        >>> list(unit_vector([]))
        []

        >>> list(unit_vector([1.0]))
        [1.0]

    Args:
        data (np.array): data to normalize
        axis (None or int): If specified, determines specific axis along data to normalize
        out (None or np.array): If specified, will store computation in this variable

    Returns:
        None or np.array: If @out is not specified, will return normalized vector. Otherwise, stores the output in @out
    """
    if out is None:
        data = np.array(data, dtype=np.float32, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(np.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = np.array(data, copy=False)
        data = out
    length = np.atleast_1d(np.sum(data * data, axis))
    np.sqrt(length, length)
    if axis is not None:
        length = np.expand_dims(length, axis)
    data /= length
    if out is None:
        return data

def rotation_matrix(angle, direction, point=None):
    """
    Returns matrix to rotate about axis defined by point and direction.

    E.g.:
        >>> angle = (random.random() - 0.5) * (2*math.pi)
        >>> direc = numpy.random.random(3) - 0.5
        >>> point = numpy.random.random(3) - 0.5
        >>> R0 = rotation_matrix(angle, direc, point)
        >>> R1 = rotation_matrix(angle-2*math.pi, direc, point)
        >>> is_same_transform(R0, R1)
        True

        >>> R0 = rotation_matrix(angle, direc, point)
        >>> R1 = rotation_matrix(-angle, -direc, point)
        >>> is_same_transform(R0, R1)
        True

        >>> I = numpy.identity(4, numpy.float32)
        >>> numpy.allclose(I, rotation_matrix(math.pi*2, direc))
        True

        >>> numpy.allclose(2., numpy.trace(rotation_matrix(math.pi/2,
        ...                                                direc, point)))
        True

    Args:
        angle (float): Magnitude of rotation
        direction (np.array): (ax,ay,az) axis about which to rotate
        point (None or np.array): If specified, is the (x,y,z) point about which the rotation will occur

    Returns:
        np.array: 4x4 homogeneous matrix that includes the desired rotation
    """
    sina = math.sin(angle)
    cosa = math.cos(angle)
    direction = unit_vector(direction[:3])
    # rotation matrix around unit vector
    R = np.array(
        ((cosa, 0.0, 0.0), (0.0, cosa, 0.0), (0.0, 0.0, cosa)), dtype=np.float32
    )
    R += np.outer(direction, direction) * (1.0 - cosa)
    direction *= sina
    R += np.array(
        (
            (0.0, -direction[2], direction[1]),
            (direction[2], 0.0, -direction[0]),
            (-direction[1], direction[0], 0.0),
        ),
        dtype=np.float32,
    )
    M = np.identity(4)
    M[:3, :3] = R
    if point is not None:
        # rotation not around origin
        point = np.array(point[:3], dtype=np.float32, copy=False)
        M[:3, 3] = point - np.dot(R, point)
    return M

def run_1d_time_optimal_control(
    x_now: float, 
    v_now: float, 
    x_final: float, 
    dt: float,
    # TODO: make these configurable
    a_max: float = 0.5, 
    d_max: float = 0.5, 
    v_max: float = 0.5,
):
    """
    Time-optimal 1D velocity command for a point-mass system.
    Assumes discrete-time kinematic model with bounded acceleration and speed.
    
    Args:
        x_now (float): current position
        v_now (float): current velocity
        x_final (float): target position
        v_final (float): target velocity (default 0.0)
        dt (float): time step
    
    Returns:
        velocity_cmd (float): target velocity for next step
    """
    # Direction-aware setup
    dx = x_final - x_now
    dir_sign = math.copysign(1.0, dx) if dx != 0 else 0.0
    speed = abs(v_now)
    dist_left = abs(dx)

    # Velocity deltas per time step
    dv_a = a_max * dt
    dv_d = d_max * dt

    # Approximate stopping distance assuming constant deceleration
    # d = v^2 / (2*a), add one dt worth of motion for conservativeness
    stopping_dist = speed * dt + (speed ** 2) / (2.0 * d_max)

    # Decide control phase
    if dist_left > 1e-4:  # far enough from goal
        if speed > v_max + 1e-4:
            # Over speed, must slow down
            phase = 'O'
            speed_next = max(0.0, speed - dv_d)
        elif stopping_dist < dist_left:
            # Can safely accelerate
            phase = 'A'
            speed_next = min(v_max, speed + dv_a)
        elif speed > 1e-4:
            # Too close to accelerate, start decelerating
            phase = 'D'
            speed_next = max(0.0, speed - dv_d)
        else:
            # Already at rest near target
            phase = 'Z'
            speed_next = 0.0
    elif speed > 1e-4:
        # Passed goal or exactly at goal, but still moving
        phase = 'X'
        speed_next = max(0.0, speed - dv_d)
    else:
        phase = 'Z'
        speed_next = 0.0

    # Direction-aware velocity command
    velocity_cmd = dir_sign * speed_next

    return velocity_cmd
