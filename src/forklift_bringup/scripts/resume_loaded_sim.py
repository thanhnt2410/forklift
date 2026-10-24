#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Resume a loaded SIM only after ros2_control hardware initialization.

Fortress may otherwise simulate free fall before the model plugin is ready.
No position reset, gravity override or physical-device access is performed.
"""
import subprocess
import time
import rclpy
from controller_manager_msgs.srv import ListControllers


def main():
    rclpy.init();node=rclpy.create_node('resume_loaded_sim')
    client=node.create_client(ListControllers,'/controller_manager/list_controllers')
    try:
        if not client.wait_for_service(timeout_sec=60):raise RuntimeError('SIM controller manager not ready')
        def control(request):
            result=subprocess.run(['ign','service','-s','/world/forklift/control',
                '--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean',
                '--timeout','5000','--req',request],capture_output=True,text=True,timeout=8)
            if result.returncode or 'true' not in result.stdout:raise RuntimeError(result.stdout+result.stderr)
        deadline=time.monotonic()+60;steps=0
        while time.monotonic()<deadline:
            future=client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node,future,timeout_sec=.15)
            if not future.done():
                # Humble serializes list_controllers behind a pending switch.
                # Advance one controller update to let that switch complete.
                if steps>=20:raise RuntimeError('SIM controller manager still blocked after safe startup steps')
                control('pause: true multi_step: 10');steps+=10
                rclpy.spin_until_future_complete(node,future,timeout_sec=.5)
                if not future.done():
                    if steps>=20:raise RuntimeError('SIM controller switch not responding')
                    control('pause: true multi_step: 10');steps+=10
                    rclpy.spin_until_future_complete(node,future,timeout_sec=1)
                if not future.done():raise RuntimeError('SIM controller manager not responding')
            states={c.name:c.state for c in future.result().controller}
            if states.get('lift_controller')=='active':break
            if states.get('lift_controller')=='inactive':
                # Controller switching requires physics updates. Only advance
                # 10 ms at a time, with the lift controller activated first.
                if steps>=20:raise RuntimeError('Lift did not activate within safe startup steps')
                time.sleep(.1);control('pause: true multi_step: 10');steps+=10
            time.sleep(.1)
        else:raise RuntimeError('SIM lift controller not ready')
        control('pause: false')
        node.get_logger().info(f'Loaded SIM resumed after lift activation; startup steps={steps}')
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
