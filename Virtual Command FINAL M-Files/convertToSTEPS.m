function J = convertToSTEPS(R)
%	R is six-element array of five angles in degrees and grip opening in inches 
%	[TR SH EL P Ro GR]. They have to be converted to motor half-step
%	values as required by the TeachMover's @STEP serial command.
%	Take note that P is measured with respect to the EL coordinate
%	frame, NOT with respect to ground.
	waypoints = size(R,1);
	J = zeros(waypoints,6);
	TR	= R(:,1);
	SH	= R(:,2);
	EL	= R(:,3);
	P	= R(:,4);
	Ro	= R(:,5);
	GR	= R(:,6);
	J(:,1) = TR*19.64;
	J(:,2) = -SH*19.64;
	J(:,3) = -(SH+EL)*11.55;
	J(:,4) = -(SH+EL+P+Ro)*4.27;
	J(:,5) = -(SH+EL+P-Ro)*4.27;
	J(:,6) = -(SH+EL)*11.55 + GR*371;
end