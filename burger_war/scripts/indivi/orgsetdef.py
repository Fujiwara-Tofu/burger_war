

def odmreset():
    self.pose_x = data.pose.pose.position.x
    self.pose_y = data.pose.pose.position.y

    del_range = 0
    distance = 0.3
    turn_angle = 90
    end = 0
    self.turn_count = 0
    self.coner_posi = self.turn_count % 4
    self.posi1 = [,]
    self.posi2 = [,]
    self.posi3 = [,]
    self.posi4 = [,]

    while end > 0:
        old_lider = self.scan.ranges[0]
        if old_lider - self.scan.ranges[0] > del_range
            x = 1
        else
            '''offset_x = xxx - self.pose_x
            offset_y = yyy - self.pose_y'''
            self.offset_posi = self.posi"self.coner_posi" - [self.pose_x,self.pose_y]
            '''↑書き方がわからないけど気持ちは伝わりますか'''

            while self.scan.ranges[0]-old_lider < distance:
                  x = -1
                  twist = Twist()
                  twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
                  twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	          self.vel_pub.publish(twist)

            while self.scan.ranges[55] - self.scan.ranges[124] > -0.1 or self.scan.ranges[55] - self.scan.ranges[124] > 0.1:
                  th = 1
                  twist = Twist()
                  twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                  twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
	          self.vel_pub.publish(twist)
            end = 1
            self.turn_count += 1


    return (offset_x, offset_y)



