<?xml version="1.0"?> 
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro"> 
	<xacro:property name="base_length" value="0.6" />
	<xacro:property name="wheel_radius" value="0.1" />
	<xacro:property name="wheel_length" value="0.05" />
	<xacro:property name="base_width" value="0.2" />
	<xacro:property name="base_height" value="0.1" />
	
	<xacro:macro name="box_inertia" params="mass lx ly lz xyz rpy"> 
		<inertial> 
			<origin xyz="${xyz}" rpy="${rpy}" /> 
			<mass value="${mass}" /> 
			<inertia ixx="${(mass / 12.0) * (ly * ly + lz * lz)}" ixy="0" ixz="0" iyy="${(mass / 12.0) * (lx * lx + lz * lz)}" iyz="0" izz="${(mass / 12.0) * (lx * lx + ly * ly)}" /> 
		</inertial> 
	</xacro:macro>
	<xacro:macro name="cylinder_inertia" params="mass r h xyz rpy"> 
		<inertial> 
			<origin xyz="${xyz}" rpy="${rpy}" /> 
			<mass value="${mass}" /> 
			<inertia ixx="${((1 / 12) * mass * (3 * (r * r * r) * (h * h)))}" ixy="0" ixz="0" iyy="${((1 / 12) * mass * (3 * (r * r * r) * (h * h)))}" iyz="0" izz="${((1 / 2) * mass * (r * r))}" /> 
		</inertial> 
	</xacro:macro>
	<xacro:macro name="sphere_inertia" params="mass r xyz rpy"> 
		<inertial> 
			<origin xyz="${xyz}" rpy="${rpy}" /> 
			<mass value="${mass}" /> 
			<inertia ixx="${((2 / 5) * mass * (r * r))}" ixy="${((2 / 5) * mass * (r * r))}" ixz="${((2 / 5) * mass * (r * r))}" iyy="${((2 / 5) * mass * (r * r))}" iyz="${((2 / 5) * mass * (r * r))}" izz="${((2 / 5) * mass * (r * r))}" /> 
		</inertial> 
	</xacro:macro>
	
	<material name="green"> 
		<color rgba="0 0.6 0 .8"/> 
	</material> 
	<material name="white"> 
		<color rgba="1 1 1 .8"/> 
	</material> 
	<material name="gray"> 
		<color rgba=".7 .7 .7 .8"/> 
	</material> 
	
	<link name="base_link"> 
		<visual> 
			<geometry> 
				<box size="${base_length} 0.4 0.2" /> 
			</geometry> 
			<origin xyz="0 0 ${base_height}" rpy="0 0 0" />  
			<material name="green"/> 
		</visual>
		<xacro:box_inertia mass="5.0" lx="${base_length}" ly="${base_width}" lz="${base_height}" xyz="0 0 ${base_height / 2.0}" rpy="0 0 0" /> 
		<collision> 
			<geometry> 
				<box size="${base_length} 0.4 0.2" /> 
			</geometry> 
			<origin xyz="0 0 ${base_height}" rpy="0 0 0" />  
		</collision>
	</link>
	<link name="lidar"> 
		<visual> 
			<geometry> 
				<cylinder radius="0.1" length="0.05" /> 
			</geometry> 
			<origin xyz="0 0 0" rpy="0 0 0" /> 
			<material name="white"/> 
		</visual> 
		<xacro:cylinder_inertia mass="1.0" h="0.05" r="0.1" xyz="0 0 0" rpy="0 0 0"/>
		<collision> 
			<geometry> 
				<cylinder radius="0.1" length="0.05" /> 
			</geometry> 
			<origin xyz="0 0 0" rpy="0 0 0" />
		</collision>
	</link>
	<link name="left_wheel"> 
		<visual> 
			<geometry> 
				<cylinder radius="${wheel_radius}" length="${wheel_length}" /> 
			</geometry> 
			<origin xyz="0 0 ${wheel_length / 2.0}" rpy="0 0 0" /> 
			<material name="gray"/> 
		</visual> 
		<xacro:cylinder_inertia mass="1.0" r="${wheel_radius}" h="${wheel_length}" xyz="0 0 ${wheel_length / 2.0}" rpy="0 0 0" /> 
		<collision> 
			<geometry> 
				<cylinder radius="${wheel_radius}" length="${wheel_length}" /> 
			</geometry> 
			<origin xyz="0 0 ${wheel_length / 2.0}" rpy="0 0 0" />
		</collision>
	</link>
	<link name="right_wheel"> 
		<visual> 
			<geometry> 
				<cylinder radius="${wheel_radius}" length="${wheel_length}" /> 
			</geometry> 
			<origin xyz="0 0 ${wheel_length / 2.0}" rpy="0 0 0" /> 
			<material name="gray"/> 
		</visual> 
		<xacro:cylinder_inertia mass="1.0" r="${wheel_radius}" h="${wheel_length}" xyz="0 0 ${wheel_length / 2.0}" rpy="0 0 0" /> 
		<collision> 
			<geometry> 
				<cylinder radius="${wheel_radius}" length="${wheel_length}" /> 
			</geometry> 
			<origin xyz="0 0 ${wheel_length / 2.0}" rpy="0 0 0" />
		</collision>
	</link>
	<link name="caster_wheel"> 
		<visual> 
			<geometry> 
				<sphere radius="${wheel_radius / 2.0}" /> 
			</geometry> 
			<origin xyz="0 0 0" rpy="0 0 0" /> 
			<material name="gray" /> 
		</visual> 
		<xacro:sphere_inertia mass="2.0" r="${wheel_radius / 2.0}" xyz="0 0 ${wheel_length / 2.0}" rpy="0 0 0" /> 
		<collision> 
			<geometry> 
				<sphere radius="${wheel_radius / 2.0}" /> 
			</geometry> 
			<origin xyz="0 0 0" rpy="0 0 0" />
		</collision>
	</link>
	<link name="base_footprint" />
	
	<joint name="base_lidar_joint" type="fixed"> 
		<parent link="base_link"/> 
		<child link="lidar"/> 
		<origin xyz="${base_length / 4.0} 0 ${base_height * 2.25}" rpy="0 0 0"/> 
	</joint>
	<joint name="base_left_wheel_joint" type="continuous"> 
		<parent link="base_link"/> 
		<child link="left_wheel"/> 
		<origin xyz="${-base_length / 4.0} ${base_width} 0" rpy="${-pi / 2.0} 0 0"/> 
		<axis xyz="0 0 1"/> 
	</joint>
	<joint name="base_right_wheel_joint" type="continuous"> 
		<parent link="base_link"/> 
		<child link="right_wheel"/> 
		<origin xyz="${-base_length / 4.0} ${-base_width} 0" rpy="1.57 0 0"/> 
		<axis xyz="0 0 1"/> 
	</joint>
	<joint name="base_caster_wheel_joint" type="fixed"> 
		<parent link="base_link"/> 
		<child link="caster_wheel"/> 
		<origin xyz="${base_length / 3.0} 0 ${-wheel_radius / 2.0}" rpy="0 0 0" />
	</joint>
	<joint name="base_joint" type="fixed"> 
		<parent link="base_footprint"/> 
		<child link="base_link"/> 
		<origin xyz="0 0 ${wheel_radius}" rpy="0 0 0" /> 
	</joint>
	
	<gazebo reference="base_link"> 
		<material>Gazebo/Green</material> 
	</gazebo>
	<gazebo reference="caster_wheel"> 
		<material>Gazebo/Gray</material> 
	</gazebo>
	<gazebo reference="left_wheel"> 
		<material>Gazebo/Gray</material> 
	</gazebo>
	<gazebo reference="right_wheel"> 
		<material>Gazebo/Gray</material> 
	</gazebo>
	<gazebo reference="lidar"> 
		<material>Gazebo/White</material> 
	</gazebo>
	
	<gazebo> 
		<plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so"> 
			<!-- Update rate in Hz--> 
			<update_rate>50</update_rate>
			
			<!-- wheels--> 
			<left_joint>base_left_wheel_joint</left_joint> 			
			<right_joint>base_right_wheel_joint</right_joint> 
			
			<!-- kinematics--> 
			<wheel_separation>0.4</wheel_separation> 
			<wheel_diameter>0.2</wheel_diameter> 
			
			<!-- output--> 
			<publish_odom>true</publish_odom> 
			<publish_odom_tf>true</publish_odom_tf> 
			<publish_wheel_tf>true</publish_wheel_tf> 
			
			<odometry_topic>odom</odometry_topic> 
			<odometry_frame>odom</odometry_frame> 
			<robot_base_frame>base_footprint</robot_base_frame> 
			
			<!-- input--> 
			<command_topic>cmd_vel</command_topic> 
		</plugin> 
	</gazebo>
</robot>