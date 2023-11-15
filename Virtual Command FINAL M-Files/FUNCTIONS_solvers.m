function [FKsolver, FKsolver_RAW, IKsolver, GRsolver, Psolver] = FUNCTIONS_solvers
%	This function will return the handles to the other functions defined in
%	this M-file. A program calling FUNCTIONS_solvers will then be able to
%	call the functions in this M-file.
	FKsolver		= @FKsolver_fun;
	FKsolver_RAW	= @FKsolver_RAW_fun;
	IKsolver		= @IKsolver_fun;
	GRsolver		= @GRsolver_fun;
	Psolver			= @Psolver_fun;
end

function coordinate_frame_position = FKsolver_fun(joint_xform,coordinate_frame_numbers)
%	SYNTAX:
%	coordinate_frame_position = FKsolver(joint_xform,coordinate_frame_numbers)
%
%	WHERE:
%	joint_xform is an array of handles to hgtransform objects
%	coordinate_frame_numbers is a row vector referring to the coordinate frame
%		numbers (see EXAMPLE below)
%	coordinate_frame_position is a 1x3 row vector corresponding to [x y z].
%		It is the location of the last coordinate frame with respect to the
%		first coordinate frame (see EXAMPLE below)
%
%	{1} Trunk coordinate frame (located at the shoulder)
%	{2} Shoulder coordinate frame (located at the elbow)
%	{3} Elbow coordinate frame (located at the wrist)
%	{4} Pitch coordinate frame (located at the wrist, coincident with Elbow coordinate frame)
%	{5} Roll coordinate frame (located at the end effector)
%	{6} Left Finger coordinate frame (located at the hinge of the left finger)
%	{7} Right Finger coordinate frame (located at the hinge of the right finger)
%	{8} Left Tip coordinate frame (located at the joint where the left tip
% 		joins the left finger)
%	{9} Right Tip coordinate frame (located at the joint where the right tip
% 		joins the right finger)
%
%	EXAMPLE:
% 	1.	To get the position of the Roll coordinate frame (also the end-effector)
% 		with respect to the Shoulder "joint" (not "coordinate frame", which is 
% 		located at the elbow joint), following the 
% 		{2}Shoulder -> {3}Elbow -> {4}Pitch -> {5}Roll 
%		serial link chain:
%		p = FKsolver(joint_xform,2:5);
% 
% 	2.	To get the position of the Right Tip coordinate frame with respect 
%		to the Shoulder joint (not coordinate frame, which is located at the 
% 		elbow joint), following the 
% 		{2}Shoulder -> {3}Elbow -> {4}Pitch -> {5}Roll -> {7}Right Finger -> {9}Right Tip
%		serial link chain:
%		p = FKsolver(joint_xform,[2 3 4 5 7 9])
%
% 	3.	To get the position of the Roll (also the end-effector) coordinate frame 
% 		with respect to the origin, following the 
% 		{1}Trunk -> {2}Shoulder -> {3}Elbow -> {4}Pitch -> {5}Roll
%		serial link chain:
%		p = FKsolver(joint_xform,1:5)
%
%	This is very useful in keeping track of the positions of each joint with 
%	respect to another joint above it in the heirarchy, for safety concerns
%	such as collision warnings.
%
%	See also <a href="matlab:help IKsolver">IKsolver</a>, <a href="matlab:help Psolver">Psolver</a>
	effector = eye(4,4);
	for m = coordinate_frame_numbers
		effector = effector*get(joint_xform(m),'Matrix');
	end; clear m
	coordinate_frame_position = effector(1:3,4)';
end

function end_effector_position = FKsolver_RAW_fun(theta_current)
%	SYNTAX
%	end_effector_position = FKsolver_RAW(theta_current)
% 
%	theta_current is a five-element array of angles in degrees. It is the
%		same as the output of IKsolver.
% 
%	This function will solve for the cartesian coordinate position of the
%	end-effector. This is the forward kinematics solution presented in the
%	TeachMover User Manual.
	L	= 1.778;
	H	= 1.95;
	LL	= 0.965;
	TR	= theta_current(1);
	SH	= theta_current(2);
	EL	= theta_current(3);
	P	= theta_current(4);

	RR	= L*cosd(SH) + L*cosd(SH+EL) + LL*cosd(P);
	X	= RR*cosd(TR);
	Y	= RR*sind(TR);
	Z	= H + L*sind(SH) + L*sind(SH+EL) + LL*sind(P);
	end_effector_position = [X Y Z];
end

function theta_new = IKsolver_fun(pos,PP,R)
%	SYNTAX:
%	theta_new = IKsolver(pos,PP,R)
%
%	This function will calculate inverse kinematics given the desired end
%	effector position (pos = [x y z]), desired pitch angle relative to ground (PP),
%	and roll (R). This is the inverse kinematics solution presented in the
%	TeachMover User Manual.
%
%	The output is a 1x5 array of angles
%	theta_new = [TR SH EL P R]
%	These angles are expressed in degrees each with respect to their parent
%	coordinate frame.
%
%	See also <a href="matlab:help Psolver">Psolver</a>, <a href="matlab:help FKsolver">FKsolver</a>
	x		= pos(1);
	y		= pos(2);
	z		= pos(3);

	L		= 1.778;
	H		= 1.95;
	LL		= 0.965;
	r		= sqrt(x^2 + y^2);
	Zo		= z - LL*sind(PP) - H;
	Ro		= r - LL*cosd(PP);
	b		= sqrt(Zo^2 + Ro^2)/2;
	alpha	= acosd(b/L);
	beta	= atan2(Zo,Ro)*180/pi;

	TR		= atan2(y,x)*180/pi;
	SH		= alpha + beta;
	EL		= -2*alpha;
	P		= PP + (alpha-beta);

	theta_new		= zeros(1,5);
	theta_new(1,1)	= TR;
	theta_new(1,2)	= SH;
	theta_new(1,3)	= EL;
	theta_new(1,4)	= P;
	theta_new(1,5)	= R;
	theta_new		= real(theta_new);

	theta_new(1)	= min(theta_new(1),90);			% Set upperbound for TR
	theta_new(1)	= max(theta_new(1),-90);		% Set lowerbound for TR
	theta_new(2)	= min(theta_new(2),144);		% Set upperbound for SH
	theta_new(2)	= max(theta_new(2),-35);		% Set lowerbound for SH
	theta_new(3)	= min(theta_new(3),0);			% Set upperbound for EL
	theta_new(3)	= max(theta_new(3),-149);		% Set lowerbound for EL
	theta_new(4)	= min(theta_new(4),80);			% Set upperbound for P
	theta_new(4)	= max(theta_new(4),-80);		% Set lowerbound for P
end

function pitch_angle = Psolver_fun(joint_xform)
%	SYNTAX
%	pitch_angle = Psolver(joint_xform)
%
%	Using forward kinematics from Shoulder to Pitch (2:4), this function will 
%	solve for the pitch angle in degrees relative to ground
%	
%	See also <a href="matlab:help IKsolver">IKsolver</a>, <a href="matlab:help FKsolver">FKsolver</a>
	T_pitch = eye(4,4);
	for m = 2:4
		T_pitch = T_pitch*get(joint_xform(m),'Matrix');
	end; clear m
	pitch_angle = atan2(T_pitch(2,3),T_pitch(1,3))*180/pi;
end

function thetaGR = GRsolver_fun(G)
%	SYNTAX
%	thetaGR = GRsolver(G)
%	
%	This function will solve for the finger angle (thetaGR) given desired 
%	Gripper opening in inches (G). This doesn't have any importance for the
% 	final TeachMover operation. However, this is useful for simulating the
%	Gripper on the 3D Model.
	L1 = 1.884;
	L2 = 1.700;
	G0 = 1.520;
	straight = 2.2835/2;
	LL = L1 + sqrt(L2^2 - ((G - G0).^2)/2) - 2.5/2.54;	
	in = G/2 < straight;
	out = ~in;
	gi = straight - G(in)/2;
	thetaGR(in) = -atan(gi./LL(in))*180/pi;
	go = G(out)/2 - straight;
	thetaGR(out) = atan(go./LL(out))*180/pi;
end