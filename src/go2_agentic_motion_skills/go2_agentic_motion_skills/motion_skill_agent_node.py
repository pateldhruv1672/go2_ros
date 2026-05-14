from __future__ import annotations

import os
import re
import threading
import time
from typing import Callable

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
except Exception as exc:
    raise RuntimeError(
        'unitree_sdk2_python is required. Install it from the official repo with pip3 install -e .'
    ) from exc


class MotionSkillAgentNode(Node):
    def __init__(self) -> None:
        super().__init__('motion_skill_agent_node')
        self.declare_parameter('network_interface', os.environ.get('UNITREE_NET_IF', ''))
        self.declare_parameter('motion_mode', 'normal')
        self.declare_parameter('allow_during_navigation', False)
        self.declare_parameter('sport_timeout_sec', 10.0)
        self.declare_parameter('motion_switch_timeout_sec', 5.0)
        self.declare_parameter('toggle_duration_sec', 4.0)

        self.navigation_active = False
        self._sdk_lock = threading.Lock()
        self._init_sdk()

        self.create_subscription(String, '/motion_skills/command', self.command_cb, 20)
        self.create_subscription(String, '/semantic_nav/status', self.status_cb, 20)
        self.reply_pub = self.create_publisher(String, '/agent/reply', 20)
        self.status_pub = self.create_publisher(String, '/motion_skills/status', 20)

        self.get_logger().info('ready | concrete Unitree SDK2 motion skills enabled')

    def _init_sdk(self) -> None:
        net_if = str(self.get_parameter('network_interface').value).strip()
        if net_if:
            ChannelFactoryInitialize(0, net_if)
        else:
            ChannelFactoryInitialize(0)

        self.sport = SportClient()
        self.sport.SetTimeout(float(self.get_parameter('sport_timeout_sec').value))
        self.sport.Init()

        self.motion_switcher = MotionSwitcherClient()
        self.motion_switcher.SetTimeout(float(self.get_parameter('motion_switch_timeout_sec').value))
        self.motion_switcher.Init()

    def status_cb(self, msg: String) -> None:
        text = (msg.data or '').lower()
        if 'sending_goal' in text or 'navigating' in text:
            self.navigation_active = True
        if 'goal succeeded' in text or 'goal failed' in text or 'goal rejected' in text or 'cancel' in text:
            self.navigation_active = False

    def publish(self, pub, text: str) -> None:
        msg = String()
        msg.data = text
        pub.publish(msg)

    def _select_mode(self) -> None:
        mode = str(self.get_parameter('motion_mode').value).strip()
        if mode:
            try:
                self.motion_switcher.SelectMode(mode)
            except Exception as exc:
                self.get_logger().warn(f'motion switch failed for mode {mode}: {exc}')

    def _normalize(self, text: str) -> str:
        t = (text or '').strip().lower().replace('-', ' ').replace('_', ' ')
        t = re.sub(r'\s+', ' ', t)
        alias_map = {
            'stand': 'stand_up',
            'stand up': 'stand_up',
            'standdown': 'stand_down',
            'stand down': 'stand_down',
            'recover': 'recovery_stand',
            'recovery': 'recovery_stand',
            'balanced stand': 'balance_stand',
            'balance stand': 'balance_stand',
            'stop': 'stop_move',
            'stop move': 'stop_move',
            'wave': 'hello',
            'jump': 'front_jump',
            'dance': 'dance1',
            'front flip': 'front_flip',
            'back flip': 'back_flip',
            'front pounce': 'front_pounce',
            'hand stand': 'hand_stand',
            'free walk': 'free_walk',
            'free bound': 'free_bound',
            'free jump': 'free_jump',
            'free avoid': 'free_avoid',
            'walk upright': 'walk_upright',
            'cross step': 'cross_step',
            'static walk': 'static_walk',
            'trot run': 'trot_run',
            'classic walk': 'classic_walk',
            'switch avoid mode': 'switch_avoid_mode',
            'auto recovery on': 'auto_recovery_on',
            'auto recovery off': 'auto_recovery_off',
        }
        return alias_map.get(t, t.replace(' ', '_'))

    def _do_timed_toggle(self, on_call: Callable[[bool], int]) -> int:
        duration = float(self.get_parameter('toggle_duration_sec').value)
        code = on_call(True)
        time.sleep(duration)
        off_code = on_call(False)
        return 0 if code == 0 and off_code == 0 else (off_code if off_code != 0 else code)

    def _execute_skill(self, skill: str) -> int:
        self._select_mode()
        actions: dict[str, Callable[[], int]] = {
            'damp': lambda: self.sport.Damp(),
            'balance_stand': lambda: self.sport.BalanceStand(),
            'stop_move': lambda: self.sport.StopMove(),
            'stand_up': lambda: self.sport.StandUp(),
            'stand_down': lambda: self.sport.StandDown(),
            'recovery_stand': lambda: self.sport.RecoveryStand(),
            'sit': lambda: self.sport.Sit(),
            'rise_sit': lambda: self.sport.RiseSit(),
            'hello': lambda: self.sport.Hello(),
            'stretch': lambda: self.sport.Stretch(),
            'content': lambda: self.sport.Content(),
            'dance1': lambda: self.sport.Dance1(),
            'dance2': lambda: self.sport.Dance2(),
            'scrape': lambda: self.sport.Scrape(),
            'front_flip': lambda: self.sport.FrontFlip(),
            'front_jump': lambda: self.sport.FrontJump(),
            'front_pounce': lambda: self.sport.FrontPounce(),
            'heart': lambda: self.sport.Heart(),
            'left_flip': lambda: self.sport.LeftFlip(),
            'back_flip': lambda: self.sport.BackFlip(),
            'free_walk': lambda: self.sport.FreeWalk(),
            'static_walk': lambda: self.sport.StaticWalk(),
            'trot_run': lambda: self.sport.TrotRun(),
            'switch_avoid_mode': lambda: self.sport.SwitchAvoidMode(),
            'auto_recovery_on': lambda: self.sport.AutoRecoverySet(True),
            'auto_recovery_off': lambda: self.sport.AutoRecoverySet(False),
            'hand_stand': lambda: self._do_timed_toggle(self.sport.HandStand),
            'free_bound': lambda: self._do_timed_toggle(self.sport.FreeBound),
            'free_jump': lambda: self._do_timed_toggle(self.sport.FreeJump),
            'free_avoid': lambda: self._do_timed_toggle(self.sport.FreeAvoid),
            'walk_upright': lambda: self._do_timed_toggle(self.sport.WalkUpright),
            'cross_step': lambda: self._do_timed_toggle(self.sport.CrossStep),
            'classic_walk': lambda: self._do_timed_toggle(self.sport.ClassicWalk),
        }
        if skill not in actions:
            raise KeyError(skill)
        return actions[skill]()

    def command_cb(self, msg: String) -> None:
        raw = (msg.data or '').strip()
        if not raw:
            return
        skill = self._normalize(raw)

        if self.navigation_active and not bool(self.get_parameter('allow_during_navigation').value):
            self.publish(self.reply_pub, 'I will not run motion skills while navigation is active.')
            return

        self.publish(self.status_pub, f'executing {skill}')
        try:
            with self._sdk_lock:
                code = self._execute_skill(skill)
            if code == 0:
                self.publish(self.reply_pub, f'Completed {skill}.')
                self.publish(self.status_pub, f'completed {skill}')
            else:
                self.publish(self.reply_pub, f'Motion skill {skill} returned code {code}.')
                self.publish(self.status_pub, f'failed {skill}: code {code}')
        except KeyError:
            self.publish(self.reply_pub, f'I do not know the motion skill {raw}.')
        except Exception as exc:
            self.publish(self.reply_pub, f'Motion skill {skill} failed: {exc}')
            self.publish(self.status_pub, f'failed {skill}: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionSkillAgentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
