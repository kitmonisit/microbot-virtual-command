function MODEL = import_3Dmodel(fh,ah)
	%% Target Indicator
	for m = 1:4
		target(m) = line(1,0,1,...
							'Parent',ah(m));
	end;	clear m
	set(target,		'Marker','x',...
					'MarkerEdgeColor',[1 0 0],...
					'MarkerSize',5,...
					'HitTest','off');
	target_link(1) = linkprop(target([1 2 4]),'XData');
	target_link(2) = linkprop(target(1:4),{	'YData',...
											'ZData',...
											'Marker',...
											'MarkerEdgeColor',...
											'MarkerFaceColor',...
											'MarkerSize',...
											'LineWidth'});

	%% Tabletop
	vert = [-1 -4.55 0;
			-1 4.55 0;
			4.55 4.55 0;
			4.55 -4.55 0];
	fac = [1 2 3 4];
	tabletop = patch(	'Parent',ah(4),...
						'Vertices',vert,...
						'Faces',fac,...
						'FaceColor',[0 .5 .5],...
						'FaceLighting','none');
	clear vert fac tabletop
	
	load patch_data
	load DH_parameters

	%% Base
	axes(ah(4))
	joint_base = patch(	'Vertices',base_vert,...
						'Faces',base_face,...
						'FaceLighting','none',...
						'FaceColor',[.3 .3 .3],...
						'HitTest','off');
	copyobj(joint_base,ah(1:3));
	clear joint_base

	%% HGTRANSFORM objects
	for n = 1:5 % Viewports
		for m = 1:9 % Coordinate frames
			joint_xform(m,n) = hgtransform('Parent',ah(n));
		end; clear m
	end; clear n
	joint_xform(5,6) = hgtransform('Parent',ah(6));

	%% HGTRANSFORM linkprops
	for m = 1:9 % Coordinate frames
		switch m
			case 1
				joint_xform_link(m) = linkprop(joint_xform(m,[1 2 4]),{'Matrix','UserData'});
			case 5
				joint_xform_link(m) = linkprop(joint_xform(m,[1 2 3 4 6]),{'Matrix','UserData'});
			otherwise
				joint_xform_link(m) = linkprop(joint_xform(m,[1 2 3 4]),{'Matrix','UserData'});
		end
	end; clear m

	%% Attach the 3D patch objects to their respective coordinate frames in every viewport
	%% {1} Trunk patch			-->			{1} Trunk coordinate frame
	%% {2} UpperArm patch		-->			{2} Shoulder coordinate frame
	%% {3} LowerArm patch		-->			{3} Elbow coordinate frame
	%% {4} NULL patch			-->			{4} Pitch coordinate frame (actually there is no 
	%%															need for a 3D model here)
	%% {5} Gripper patch		-->			{5} Roll coordinate frame (Gripper patch is 
	%%															parented to Roll because 
	%%															when Roll rigidly follows
	%%															Pitch, Gripper patch can
	%%															visualize both Pitch and Roll)
	%% {6} Left Finger patch	-->			{6} Left Finger coordinate frame
	%% {7} Right Finger patch	-->			{7} Right Finger coordinate frame
	%% {8} Left Tip patch		-->			{8} Left Tip coordinate frame
	%% {9} Right Tip patch		-->			{9} Right Tip coordinate frame
	%%
	%% At ah(5), Pitch viewport, Gripper patch is parented to Pitch coordinate frame 
	%% because there is no need to visualize Roll in this viewport.
	light_h = zeros(1,4);
	for n = 5:-1:1 % Viewports
		for m = 1:9 % Robot Links
			if n <= 4
				joint_patch(m,n) = patch(	'Parent',joint_xform(m,n),...
											'Vertices',vert{m},...
											'Faces',face{m});
			elseif n == 5 && m == 5
				joint_patch(m,n) = patch(	'Parent',joint_xform(4,5),...
											'Vertices',vert{m},...
											'Faces',face{m});
			elseif n == 5 && any(m == 6:9)
				joint_patch(m,n) = patch(	'Parent',joint_xform(m,5),...
											'Vertices',vert{m},...
											'Faces',face{m});
			end
		end; clear m
		if any(n == 1:3)
			light_h(n) = light(				'Parent',ah(n),...
											'Position',[2 0 5],...
											'Style','Infinite');
		end
	end; clear n
	mask = joint_patch ~= 0;
	set(joint_patch(mask),	'FaceLighting','flat',...
							'FaceColor',[.5 .5 .5],...
							'EdgeColor','none',...
							'SpecularExponent',25,...
							'SpecularStrength',1,...
							'DiffuseStrength',.7,...
							'AmbientStrength',.5);
	clear mask

	roll_model(1) = line([0 0],[0 0],[0 0],		'Marker','s',...
												'MarkerFaceColor',[0 .8 0]);
	roll_model(2) = line(0,1.5,0,				'Marker','o');
	set(roll_model,								'Parent',joint_xform(5,6),...
												'MarkerSize',12,...
												'Color',[0 1 0],...
												'HitTest','off')
	set(roll_model(2),							'MarkerSize',10)

	%% Setup model
	for m = 1:5
		A = makehgtform('zrotate',DH(m,1));
		B = makehgtform('translate',[0 0 DH(m,2)]);
		C = makehgtform('translate',[DH(m,3) 0 0]);
		D = makehgtform('xrotate',DH(m,4));
		T{m} = A*B*C*D;
	end; clear m A B C D
	
	for m = 1:5
		jointData.homeMatrix = T{m};
		jointData.currentMatrix = jointData.homeMatrix;
		jointData.currentAngle = 0;
        set(joint_xform(m,4),'Matrix',T{m},...
							 'UserData',jointData)
	end; clear m
	jointData.homeMatrix = T{4};
	jointData.currentMatrix = jointData.homeMatrix;
	jointData.currentAngle = 0;
	set(joint_xform(4,5),		'Matrix',T{4},...
								'UserData',jointData)
	jointData.homeMatrix = T{5};
	jointData.currentMatrix = jointData.homeMatrix;
	jointData.currentAngle = 0;
	set(joint_xform(5,6),		'Matrix',T{5},...
								'UserData',jointData)
	line([0 0],[0 0],[0 0.75],	'Parent',joint_xform(4,5),...
								'Color',[0 1 0],...
								'HitTest','off',...
								'Marker','o',...
								'MarkerSize',10,...
								'LineWidth',1)
    xcorrect = makehgtform('xrotate',2*pi);
    set(joint_xform(1,3),'Matrix',xcorrect*T{1})
    clear xcorrect

	for m = 2:5
		for n = [1 2 3 4]
			set(joint_xform(m,n),'Parent',joint_xform(m-1,n))
			set(joint_xform(6:7,n),'Parent',joint_xform(5,n))
			set(joint_xform(8,n),'Parent',joint_xform(6,n))
			set(joint_xform(9,n),'Parent',joint_xform(7,n))
		end; clear n
	end; clear m

	LF = makehgtform('translate',[0 -0.28 -0.725])*makehgtform('yrotate',pi/2)*makehgtform('zrotate',pi);
	LF(abs(0-LF)<eps) = 0;
	RF = LF;
	RF(2,4) = -LF(2,4);
	Z = 16;
	set(joint_xform(6,4:5),'Matrix',LF*makehgtform('zrotate',-Z*pi/180));
	set(joint_xform(7,4:5),'Matrix',RF*makehgtform('zrotate',Z*pi/180));
	LT = makehgtform('translate',[0.4 0.2 0]);
	RT = makehgtform('translate',[0.4 -0.2 0]);
	set(joint_xform(8,4:5),'Matrix',LT*makehgtform('zrotate',Z*pi/180));
	set(joint_xform(9,4:5),'Matrix',RT*makehgtform('zrotate',-Z*pi/180));
	clear LF RF LT RT Z
	
	for m = 6:9
		jointData.homeMatrix = get(joint_xform(m,5),'Matrix');
		jointData.currentOpening = 0;
		set(joint_xform(m,4),'UserData',jointData)
	end; clear m

	set(joint_xform(6:7,5),'Parent',joint_xform(4,5))
	set(joint_xform(8,5),'Parent',joint_xform(6,5))
	set(joint_xform(9,5),'Parent',joint_xform(7,5))
	
	set(joint_patch(1:3,1),		'FaceAlpha',.3,...
								'FaceColor',[192/255 177/255 116/255]+.1,...
								'FaceLighting','gouraud');
    set(joint_patch(1:3,2:3),	'FaceColor',[192/255 177/255 116/255]+.1,...
								'FaceLighting','gouraud');
	set(joint_patch(1:3,4),		'FaceColor',[192/255 177/255 116/255]+.1,...
								'FaceLighting','gouraud',...
								'EdgeColor',[.5 .5 .5]);
	set(joint_patch(5:9,1),		'FaceAlpha',.3,...
								'FaceColor',[.1 .1 .1],...
								'FaceLighting','none');
	set(joint_patch(5:9,2:5),	'FaceColor',[.1 .1 .1],...
								'FaceLighting','none');
	clear base_face base_vert vert face DH

	%%	Store all graphics handles into MODEL structure
	MODEL.frames = joint_xform;
	MODEL.framesLinks = joint_xform_link;
	MODEL.target = target;
	MODEL.targetLinks = target_link;
	MODEL.roll = roll_model;
end