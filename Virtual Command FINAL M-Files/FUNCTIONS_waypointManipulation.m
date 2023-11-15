function waypointComputeRelative = FUNCTIONS_waypointManipulation
%	This function will return the handles to the other functions defined in
%	this M-file. A program calling FUNCTIONS_waypointManipulation will then
% 	be able to call the functions in this M-file.
	waypointComputeRelative = @waypointComputeRelative_Fcn;
end

function waypoint_Relative = waypointComputeRelative_Fcn(waypoint_RAW)
%	This function will manipulate the waypoint list such that the values in
%	each row will be relative to the values in the previous row. This is
%	useful in formulating the @STEP serial command, since it relies on
%	relative values (NOT absolute).
	waypoint_RAW = waypoint_RAW(:,1:6);
	waypoints = size(waypoint_RAW,1);
	waypoint_Relative = zeros(waypoints,6);
	for m = 2:waypoints;
		waypoint_Relative(m,:) = waypoint_RAW(m,:) - waypoint_RAW(m-1,:);	
	end; clear m
end