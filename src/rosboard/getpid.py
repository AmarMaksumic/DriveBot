import psutil
import subprocess
# 根据进程名获取进程 PID
def get_pid_by_process_name(process_name):
    pid = None
    for p in psutil.process_iter(['name', 'pid']):
        print(p)
        if p.info['name'] == process_name:
            pid = p.info['pid']
            break
    return pid

# 示例：获取 Chrome 浏览器的 PID
chrome_pid = get_pid_by_process_name('async_slam_toolbox_node')
print(chrome_pid)


command = 'ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: \'data/maps/sdw.pbstream\'}"'
result = subprocess.run(command, shell=True, capture_output=True)
print(result.stdout.decode('utf-8'))
