"""One lossless PolyTraj representation for DDS and compatibility RPC readers."""
def trajectory_record(message, received_ns):
    record = {key:list(getattr(message,key)) for key in
              ('duration','coef_x','coef_y','coef_z','hover_p')}
    record.update(traj_id=message.traj_id, hover=message.hover, order=message.order,
                  yaw=float(message.yaw),
                  start_time=[message.start_time.sec,message.start_time.nanosec],
                  generated_at=list(divmod(received_ns,1_000_000_000)))
    return record
