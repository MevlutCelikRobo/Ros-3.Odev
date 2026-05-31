#!/usr/bin/env python3
import rospy
import math
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# ===== AYARLAR =====
if len(sys.argv) == 3:
    HEDEF_X = float(sys.argv[1])
    HEDEF_Y = float(sys.argv[2])
else:
    HEDEF_X = 1.7581768035888672
    HEDEF_Y = 0.9078540205955505

GUVENLI_MESAFE = 0.4
HEDEF_TOLERANS = 0.2
# ===================

class MoveStopRotate:
    def __init__(self):
        rospy.init_node('move_stop_rotate')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.on_mesafe = float('inf')
        self.sol_mesafe = float('inf')
        self.sag_mesafe = float('inf')
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_aci = 0.0
        self.durum = "ILERI"  # Robot durumu

        rospy.loginfo("Move-Stop-Rotate Başladı!")

    def laser_callback(self, msg):
        ranges = list(msg.ranges)
        toplam = len(ranges)

        def temizle(deger):
            if math.isinf(deger) or math.isnan(deger) or deger == 0.0:
                return float('inf')
            return deger

        on = [temizle(ranges[i]) for i in range(0, 20)] + \
             [temizle(ranges[i]) for i in range(toplam-20, toplam)]
        sol = [temizle(ranges[i]) for i in range(20, 90)]
        sag = [temizle(ranges[i]) for i in range(toplam-90, toplam-20)]

        self.on_mesafe = min(on)
        self.sol_mesafe = min(sol)
        self.sag_mesafe = min(sag)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.robot_aci = euler_from_quaternion(q)

    def hedefe_uzaklik(self):
        return math.sqrt(
            (HEDEF_X - self.robot_x) ** 2 +
            (HEDEF_Y - self.robot_y) ** 2
        )

    def hedefe_aci_farki(self):
        hedef_aci = math.atan2(
            HEDEF_Y - self.robot_y,
            HEDEF_X - self.robot_x
        )
        fark = hedef_aci - self.robot_aci
        while fark > math.pi:
            fark -= 2 * math.pi
        while fark < -math.pi:
            fark += 2 * math.pi
        return fark

    def dur(self, sure=0.5):
        """Robotu durdur"""
        twist = Twist()
        self.cmd_pub.publish(twist)
        rospy.sleep(sure)

    def don(self, yon, sure):
        """Belirli süre dön"""
        rate = rospy.Rate(10)
        twist = Twist()
        twist.angular.z = yon * 0.4
        bitis = rospy.Time.now() + rospy.Duration(sure)
        while rospy.Time.now() < bitis and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            rate.sleep()

    def calistir(self):
        rate = rospy.Rate(10)
        rospy.sleep(1)

        while not rospy.is_shutdown():
            twist = Twist()

            # Hedefe ulaştı mı?
            if self.hedefe_uzaklik() < HEDEF_TOLERANS:
                rospy.loginfo("✅ Hedefe ulaşıldı!")
                self.dur()
                break

            # ENGEL VAR
            if self.on_mesafe < GUVENLI_MESAFE:
                rospy.loginfo("🛑 Engel algılandı! Duruluyor...")
                self.dur(0.5)

                # Hangi taraf daha açık?
                if self.sol_mesafe >= self.sag_mesafe:
                    yon = 1.0
                    rospy.loginfo("⬅️ Sola dönülüyor...")
                else:
                    yon = -1.0
                    rospy.loginfo("➡️ Sağa dönülüyor...")

                # Engel geçene kadar dön
                while self.on_mesafe < GUVENLI_MESAFE and not rospy.is_shutdown():
                    twist.linear.x = 0.0
                    twist.angular.z = yon * 0.4
                    self.cmd_pub.publish(twist)
                    rate.sleep()

                # Engel geçtikten sonra 1.5 saniye daha dön
                rospy.loginfo("🔄 Engel geçildi, biraz daha dönülüyor...")
                self.don(yon, 1.5)

                # Biraz ileri git
                rospy.loginfo("➡️ Biraz ileri gidiliyor...")
                ileri = Twist()
                ileri.linear.x = 0.2
                bitis = rospy.Time.now() + rospy.Duration(1.0)
                while rospy.Time.now() < bitis and not rospy.is_shutdown():
                    self.cmd_pub.publish(ileri)
                    rate.sleep()

                rospy.loginfo("✅ Engel tamamen geçildi, hedefe devam!")

            # YOL AÇIK
            else:
                aci_farki = self.hedefe_aci_farki()

                if abs(aci_farki) > 0.15:
                    twist.linear.x = 0.05
                    twist.angular.z = 0.4 if aci_farki > 0 else -0.4
                else:
                    twist.linear.x = 0.2
                    twist.angular.z = 0.0

                self.cmd_pub.publish(twist)

            rate.sleep()

if __name__ == '__main__':
    try:
        robot = MoveStopRotate()
        robot.calistir()
    except rospy.ROSInterruptException:
        pass
