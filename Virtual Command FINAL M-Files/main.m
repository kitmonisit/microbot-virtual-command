function main
close all; clear all; clc;
format compact; format short g;
	%% Build GUI environment for positioning
	GUI = build_gui_positioning;

	%% Build GUI environment for recording
	RECORD = build_gui_recording(GUI.fig);

	%% Build GUI environment for implementation
	IMPLEMENT = build_gui_implementation(GUI.fig);

	%% Import 3D model
	MODEL = import_3Dmodel(GUI.fig,GUI.viewports);

	%% Define function handles -- SOLVERS
	[FKsolver, FKsolver_RAW, IKsolver, GRsolver, Psolver] = FUNCTIONS_solvers;

	%% Define function handles -- WAYPOINT MANIPULATION
	[waypointComputeRelative] = FUNCTIONS_waypointManipulation;

	%% Define function handles -- POSITIONING
	clickDown				= @clickDown_Fcn;
	clickDrag				= @clickDrag_Fcn;
	clickUp					= @clickUp_Fcn;
	getCurrentPoint			= @getCurrentPoint_Fcn;
	responseSlider			= @responseSlider_Fcn;
	responseTextbox			= @responseTextbox_Fcn;
	responseViewport		= @responseViewport_Fcn;
	updateApplicationData	= @updateApplicationData_Fcn;
	updateModel				= @updateModel_Fcn;
	updateSliders			= @updateSliders_Fcn;
	updateTarget			= @updateTarget_Fcn;
	updateTextboxes			= @updateTextboxes_Fcn;

	%% Define function handles -- RECORDING
	responseAddWaypoint		= @responseAddWaypoint_Fcn;
	responseReduceWaypoint	= @responseReduceWaypoint_Fcn;
	responseSetWaypoint		= @responseSetWaypoint_Fcn;
	responseListbox			= @responseListbox_Fcn;
	responseNewSequence		= @responseNewSequence_Fcn;
	responseLoadSequence	= @responseLoadSequence_Fcn;
	responseSaveSequence	= @responseSaveSequence_Fcn;
	updateActiveWaypoint	= @updateActiveWaypoint_Fcn;
	updateTotalWaypoint		= @updateTotalWaypoint_Fcn;
	updateListbox			= @updateListbox_Fcn;

	%% Define function handles -- PLAYBACK
	responseAnimate			= @responseAnimate_Fcn;

	%% Define function handles -- TESTING
	testCollision			= @testCollision_Fcn;
	testTarget				= @testTarget_Fcn;

	%% Define function handles -- IMPLEMENTATION
	ONLINE_Single			= @ONLINE_Single_Fcn;
	ONLINE_All				= @ONLINE_All_Fcn;
	OFFLINE					= @OFFLINE_Fcn;
	openPort				= @openPort_Fcn;
	closePort				= @closePort_Fcn;

	%% Define function handles -- SPECIAL
	goHome					= @goHome_Fcn;
	initializeListbox		= @initializeListbox_Fcn;
	makeSimLikeActual		= @makeSimLikeActual_Fcn;

	%% Initialize application data variables
	%% Application data variables store centralized information which is
	%% used by all functions and GUI components. All visual updates are
	%% based on this centralized data.
	click				= false;
	home_config			= [0 111.8051 -132.7571 -69.0479 0 0];
	home_settings		= [0 111.8051 -132.7571 -90 0 1 0 2 0];
	home_listed_small	= [home_config(1:6) home_settings(6:8)];
	home_listed			= repmat(home_listed_small,50,1);
	home_relative		= waypointComputeRelative(home_listed(:,1:6));
	setappdata(GUI.fig,'CurrentSettings',zeros(1,9))
	setappdata(GUI.fig,'CurrentConfigurationAngleSIM',zeros(1,6))
	setappdata(GUI.fig,'CurrentConfigurationAngleTM',zeros(1,6))
	setappdata(GUI.fig,'TargetReached',false)
	setappdata(GUI.fig,'Collision',false)
	setappdata(GUI.fig,'EnableImplementation',false)
	setappdata(GUI.fig,'AnimationInProgress',false)
	setappdata(GUI.fig,'WaypointActive',1)
	setappdata(GUI.fig,'WaypointTotal',1)
	setappdata(GUI.fig,'WaypointRAW',home_listed)
	setappdata(GUI.fig,'WaypointRELATIVE',home_relative)
	setappdata(GUI.fig,'WaypointLIST',{})
	setappdata(GUI.fig,'WaypointSTEPS',zeros(50,6))
	setappdata(GUI.fig,'WaypointQWRITE',zeros(50,6))

	%% Initialize robot data
	goHome()
	initializeListbox()

	%% Set GUI component callbacks
	set(GUI.HOME,					'Callback',goHome)
	set(GUI.fig,					'WindowButtonUpFcn',clickUp,...
									'WindowButtonMotionFcn',clickDrag)
	set(GUI.sliders,				'ButtonDownFcn',clickDown)
	set(GUI.viewports([1:3 5:6]),	'ButtonDownFcn',clickDown)
	set(GUI.textboxes,				'Callback',responseTextbox)

	%% Set RECORD component callbacks
	set(RECORD.addWaypoint,			'Callback',responseAddWaypoint)
	set(RECORD.reduceWaypoint,		'Callback',responseReduceWaypoint)
	set(RECORD.listbox,				'Callback',responseListbox)
	set(RECORD.setWaypoint,			'Callback',responseSetWaypoint)
	set(RECORD.newSequence,			'Callback',responseNewSequence)
	set(RECORD.loadSequence,		'Callback',responseLoadSequence)
	set(RECORD.saveSequence,		'Callback',responseSaveSequence)
	
	%% Set PLAYBACK component callbacks
	set(RECORD.playbackWaypoint,	'Callback',responseAnimate)

	%% Set IMPLEMENT component callbacks
	set(IMPLEMENT.onlineSingle,		'Callback',ONLINE_Single)
	set(IMPLEMENT.onlineAll,		'Callback',ONLINE_All)
	set(IMPLEMENT.offline,			'Callback',OFFLINE)
	
%% Helper functions -- POSITIONING
	function clickDown_Fcn(object,eventdata)
		if ~getappdata(GUI.fig,'AnimationInProgress')
			click = true;
			if any(gca == GUI.sliders)
				responseSlider('clickdrag');
			elseif any(gca == GUI.viewports([1:3 5:6]))
				responseViewport();
			end
		end
	end

	function clickDrag_Fcn(object,eventdata)
		if click
			if any(gca == GUI.sliders)
				responseSlider('clickdrag');
			elseif any(gca == GUI.viewports([1:3 5:6]))
				responseViewport();
			end
		end
	end

	function clickUp_Fcn(object,eventdata)
		if ~getappdata(GUI.fig,'AnimationInProgress')
			click = false;
			if any(gca == GUI.sliders)
				responseSlider('clickup')
			elseif any(gca == GUI.viewports([1:3 5:6]))
				responseViewport();
			end
		end
	end

	function [x,y,z] = getCurrentPoint_Fcn(mode)
		C = single(get(gca,'CurrentPoint'));
		x = C(1,1);
		y = C(1,2);
		z = C(1,3);
		switch mode
			case 'slider'
				setlimits
			case 'viewport'
				viewport_number = find(gca == GUI.viewports);
				current_settings = getappdata(GUI.fig,'CurrentSettings');
				switch viewport_number
					case 1		% Top view
						z = current_settings(8);
						setlimits
					case 2		% Front view
						x = current_settings(6);
						setlimits
					case 3		% Lateral view
						x = max(x,eps*2);
						y = x*sind(current_settings(1));
						x = x*cosd(current_settings(1));
						setlimits
					case 5		% Pitch
						current_settings(4) = atan2(y,x)*180/pi;
						x = current_settings(6);
						y = current_settings(7);
						z = current_settings(8);
						setlimits
					case 6		% Roll and Grip
						current_settings(5) = atan2(y,x)*180/pi;
						grip_opening = sqrt(sum([x y].^2));
						grip_opening = min(grip_opening,3);
						grip_opening = max(grip_opening,0);
						current_settings(9) = grip_opening;
						x = current_settings(6);
						y = current_settings(7);
						z = current_settings(8);
						setlimits
				end
				setappdata(GUI.fig,'CurrentSettings',current_settings)
		end
		function setlimits
			switch mode
				case 'slider'
					limits = get(gca,'YLim');
					y = max(y,limits(1));
					y = min(y,limits(2));
				case 'viewport'
					x = max(x,eps);
					x = min(x,4.55);
					y = max(y,-4.55);
					y = min(y,4.55);
					z = max(z,eps);
					z = min(z,7);
			end
		end
	end

	function responseSlider_Fcn(mode)
		[x,y,z] = getCurrentPoint('slider');
		index = find(gca == GUI.sliders);
		switch mode
			case 'clickup'
				set(GUI.markers(index),'MarkerFaceColor',[0 .5 0])
			case 'clickdrag'
				set(GUI.markers(index),'YData',y)
				set(GUI.markers(index),'MarkerFaceColor',[0 .8 0])
		end
		if any(index == 1:3)
			updateApplicationData('FK slider')
			updateModel('FK')
		elseif any(index == 4:9)
			updateApplicationData('IK slider')
			updateModel('IK')
		end
		updateTextboxes()
		updateTarget()
		testTarget()
		testCollision()
	end

	function responseTextbox_Fcn(object,eventdata)
		index = find(object == GUI.textboxes);
		limits = get(GUI.sliders(index),'YLim');
		oldvalue = get(GUI.markers(index),'YData');
		str = get(object,'String');
		value = str2double(str);
		if any([isnan(value) ~isreal(value)])
			errordlg('Invalid input: Only numeric characters are accepted','Invalid input','modal')
			set(object,'String',num2str(oldvalue,'%0.1f'))
			return
		end
		if value < limits(1) || value > limits(2)
			str2 = sprintf('Please enter a value between %0.1f and %0.1f',limits(1),limits(2));
			errordlg(str2,'Out of range','modal')
			value = oldvalue;
		end
		set(object,'String',num2str(value,'%0.1f'))
		if any(index == 1:3)
			updateApplicationData('FK textbox')
			updateModel('FK')
		elseif any(index == 4:9)
			updateApplicationData('IK textbox')
			updateModel('IK')
		end
		updateSliders()
		updateTarget()
		testTarget()
		testCollision()
	end

	function responseViewport_Fcn
		if isempty(gco) || ~any(gco == GUI.viewports)
			return
		end
		[x,y,z] = getCurrentPoint('viewport');
		set(MODEL.target(1),'XData',x)
		set(MODEL.target(1),'YData',y)
		set(MODEL.target(1),'ZData',z)
		set(MODEL.target(3),'XData',sqrt(sum([x y].^2)))
		updateApplicationData('IK viewport')
		updateModel('IK')
		updateSliders()
		updateTextboxes()
		testTarget()
		testCollision()
	end

	function updateApplicationData_Fcn(mode)
		switch mode
			case 'FK slider'
				current_settings = cell2mat(get(GUI.markers(1:9),'YData'))';
				current_config = getappdata(GUI.fig,'CurrentConfigurationAngleSIM');
				current_settings(4) = current_config(4) + sum(current_settings(2:3));
				current_settings(6:8) = FKsolver_RAW(current_settings(1:4));
				for m = [4 6:9]
					set(GUI.markers(m),'YData',current_settings(m))
				end; clear m
				setappdata(GUI.fig,'CurrentSettings',current_settings)
				for m = [1:3 5]
					current_config(m) = current_settings(m);
				end; clear m
				setappdata(GUI.fig,'CurrentConfigurationAngleSIM',current_config)
			case 'FK textbox'
				current_settings = str2num(char(get(GUI.textboxes,'String')))';
				current_config = getappdata(GUI.fig,'CurrentConfigurationAngleSIM');
				current_settings(4) = current_config(4) + sum(current_settings(2:3));
				current_settings(6:8) = FKsolver_RAW(current_settings(1:4));
				for m = [4 6:9]
					set(GUI.textboxes(m),'String',num2str(current_settings(m),'%0.1f'))
				end; clear m
				setappdata(GUI.fig,'CurrentSettings',current_settings)
				for m = [1:3 5]
					current_config(m) = current_settings(m);
				end; clear m
				setappdata(GUI.fig,'CurrentConfigurationAngleSIM',current_config)
			case 'IK slider'
				current_settings = cell2mat(get(GUI.markers(1:9),'YData'))';
				theta_current = IKsolver(current_settings(6:8),current_settings(4),current_settings(5));
				current_settings([1:3 5]) = theta_current([1:3 5]);
				setappdata(GUI.fig,'CurrentSettings',current_settings)
				setappdata(GUI.fig,'CurrentConfigurationAngleSIM',[theta_current current_settings(9)])
				for m = 1:5
					set(GUI.markers(m),'YData',current_settings(m))
				end; clear m
			case 'IK textbox'
				current_settings = str2num(char(get(GUI.textboxes,'String')))';
				theta_current = IKsolver(current_settings(6:8),current_settings(4),current_settings(5));
				current_settings([1:3 5]) = theta_current([1:3 5]);
				setappdata(GUI.fig,'CurrentSettings',current_settings)
				setappdata(GUI.fig,'CurrentConfigurationAngleSIM',[theta_current current_settings(9)])
				for m = 1:5
					set(GUI.textboxes(m),'String',num2str(current_settings(m),'%0.1f'))
				end; clear m
			case 'IK viewport'
				current_settings = getappdata(GUI.fig,'CurrentSettings');
				current_settings(6) = get(MODEL.target(1),'XData');
				current_settings(7) = get(MODEL.target(1),'YData');
				current_settings(8) = get(MODEL.target(1),'ZData');
				theta_current = IKsolver(current_settings(6:8),current_settings(4),current_settings(5));
				current_settings([1:3 5]) = theta_current([1:3 5]);
				setappdata(GUI.fig,'CurrentSettings',current_settings)
				setappdata(GUI.fig,'CurrentConfigurationAngleSIM',[theta_current current_settings(9)])
			case 'IK direct'
				current_settings = getappdata(GUI.fig,'CurrentSettings');
				theta_current = IKsolver(current_settings(6:8),current_settings(4),current_settings(5));
				current_settings([1:3 5]) = theta_current([1:3 5]);
				setappdata(GUI.fig,'CurrentSettings',current_settings)
				setappdata(GUI.fig,'CurrentConfigurationAngleSIM',[theta_current current_settings(9)])
			case 'add waypoint'
				waypoint_total = getappdata(GUI.fig,'WaypointTotal');
				waypoint_total = waypoint_total + 1;
				if waypoint_total > 50
					errordlg('Maximum number of 50 waypoints reached','Waypoint limit','modal')
					waypoint_total = 50;
				end
				setappdata(GUI.fig,'WaypointActive',waypoint_total)
				setappdata(GUI.fig,'WaypointTotal',waypoint_total)
			case 'reduce waypoint'
				waypoint_total = getappdata(GUI.fig,'WaypointTotal');
				if waypoint_total == 1
					return
				end
				waypoint_total = waypoint_total - 1;
				setappdata(GUI.fig,'WaypointTotal',waypoint_total);
			case 'set waypoint'
				current_settings = getappdata(GUI.fig,'CurrentSettings');
				current_config = getappdata(GUI.fig,'CurrentConfigurationAngleSIM');
				waypoint_active = getappdata(GUI.fig,'WaypointActive');
				waypoint_total = getappdata(GUI.fig,'WaypointTotal');
				if waypoint_active == 1
					errordlg('Waypoint 01 must be home position. It cannot be changed. Press the + button to add a second waypoint.','Safety precaution','modal')
					return
				end
				waypoint_RAW = getappdata(GUI.fig,'WaypointRAW');
				toBeListed = [current_config(1:6) current_settings(6:8)];
				waypoint_RAW(waypoint_active,:) = toBeListed;
				if waypoint_active == waypoint_total
					responseAddWaypoint();
				end
				setappdata(GUI.fig,'WaypointRAW',waypoint_RAW)
			case 'listbox'
				selected = get(RECORD.listbox,'Value');
				if selected < 3
					set(RECORD.listbox,'Value',3)
					selected = 3;
				end
				waypoint_active = selected - 2;
				current_settings = getappdata(GUI.fig,'CurrentSettings');
				current_config = getappdata(GUI.fig,'CurrentConfigurationAngleSIM');
				waypoint_RAW = getappdata(GUI.fig,'WaypointRAW');
				current_settings([1:3 5 9]) = waypoint_RAW(waypoint_active,[1:3 5:6]);
				current_settings(4) = waypoint_RAW(waypoint_active,4) + sum(current_settings(2:3));
				current_settings(6:8) = waypoint_RAW(waypoint_active,7:9);
				setappdata(GUI.fig,'WaypointActive',waypoint_active)
				setappdata(GUI.fig,'CurrentConfigurationAngleSIM',waypoint_RAW(waypoint_active,1:6))
				setappdata(GUI.fig,'CurrentSettings',current_settings)
		end
		waypoint_total = getappdata(GUI.fig,'WaypointTotal');
		waypoint_RAW = getappdata(GUI.fig,'WaypointRAW');
		waypoint_RELATIVE = waypointComputeRelative(waypoint_RAW(1:waypoint_total,:));
		setappdata(GUI.fig,'WaypointRELATIVE',waypoint_RELATIVE)
	end

	function updateTextboxes_Fcn
		current_settings = getappdata(GUI.fig,'CurrentSettings');
		for m = 1:9
			set(GUI.textboxes(m),'String',num2str(current_settings(m),'%0.1f'))
		end; clear m
	end

	function updateSliders_Fcn
		current_settings = getappdata(GUI.fig,'CurrentSettings');
		for m = 1:9
			set(GUI.markers(m),'YData',current_settings(m))
		end; clear m
	end

	function updateTarget_Fcn
		current_settings = getappdata(GUI.fig,'CurrentSettings');
		set(MODEL.target(1),'XData',current_settings(6))
		set(MODEL.target(1),'YData',current_settings(7))
		set(MODEL.target(1),'ZData',current_settings(8))
		set(MODEL.target(3),'XData',sqrt(sum((current_settings(6:7).^2))))
	end

	function updateModel_Fcn(mode)
		current_settings = getappdata(GUI.fig,'CurrentSettings');
		switch mode
			case 'FK'
				if any(gca == GUI.sliders)
					index = find(gca == GUI.sliders);
				elseif any(gco == GUI.textboxes)
					index = find(gco == GUI.textboxes);
				end
				if any(index == [1:3 5])
					jointData = get(MODEL.frames(index,4),'UserData');
					Rz = makehgtform('zrotate',current_settings(index)*pi/180);
					jointData.currentMatrix = Rz*jointData.homeMatrix;
					jointData.currentAngle = current_settings(index);
					set(MODEL.frames(index,4),	'Matrix',jointData.currentMatrix,...
												'UserData',jointData)
				end
				jointData = get(MODEL.frames(4,5),'UserData');
				Rz = makehgtform('zrotate',current_settings(4)*pi/180);
				jointData.currentMatrix = Rz*jointData.homeMatrix;
				jointData.currentAngle = current_settings(4);
				set(MODEL.frames(4,5),	'Matrix',jointData.currentMatrix,...
										'UserData',jointData)
			case 'IK'
				theta = IKsolver(current_settings(6:8),current_settings(4),current_settings(5));
				for m = 1:5
					jointData = get(MODEL.frames(m,4),'UserData');
					Rz = makehgtform('zrotate',theta(m)*pi/180);
					jointData.currentMatrix = Rz*jointData.homeMatrix;
					jointData.currentAngle = theta(m);
					set(MODEL.frames(m,4),	'Matrix',jointData.currentMatrix,...
											'UserData',jointData)
				end; clear m
				thetaGR = GRsolver(current_settings(9)) - GRsolver(3);
				Rz = makehgtform('zrotate',thetaGR*pi/180);
				for m = [6 9]
					jointData = get(MODEL.frames(m,4),'UserData');
					jointData.currentMatrix = jointData.homeMatrix*Rz;
					jointData.currentAngle = thetaGR;
					set(MODEL.frames(m,4),	'Matrix',jointData.currentMatrix,...
											'UserData',jointData)
				end; clear m
				for m = [7 8]
					jointData = get(MODEL.frames(m,4),'UserData');
					jointData.currentMatrix = jointData.homeMatrix*inv(Rz);
					jointData.currentAngle = -thetaGR;
					set(MODEL.frames(m,4),	'Matrix',jointData.currentMatrix,...
											'UserData',jointData)
				end; clear m
				set(MODEL.roll(1),'XData',[-current_settings(9)/2-.25 current_settings(9)/2+.25])
				jointData = get(MODEL.frames(4,5),'UserData');
				Rz = makehgtform('zrotate',current_settings(4)*pi/180);
				jointData.currentMatrix = Rz*jointData.homeMatrix;
				jointData.currentAngle = current_settings(4);
				set(MODEL.frames(4,5),	'Matrix',jointData.currentMatrix,...
										'UserData',jointData)
			case 'IK direct'
				current_config = getappdata(GUI.fig,'CurrentConfigurationAngleSIM');
				for m = 1:5
					jointData = get(MODEL.frames(m,4),'UserData');
					Rz = makehgtform('zrotate',current_config(m)*pi/180);
					jointData.currentMatrix = Rz*jointData.homeMatrix;
					jointData.currentAngle = current_config(m);
					set(MODEL.frames(m,4),	'Matrix',jointData.currentMatrix,...
											'UserData',jointData)
				end; clear m
				thetaGR = GRsolver(current_config(6)) - GRsolver(3);
				Rz = makehgtform('zrotate',thetaGR*pi/180);
				for m = [6 9]
					jointData = get(MODEL.frames(m,4),'UserData');
					jointData.currentMatrix = jointData.homeMatrix*Rz;
					jointData.currentAngle = thetaGR;
					set(MODEL.frames(m,4),	'Matrix',jointData.currentMatrix,...
											'UserData',jointData)
				end; clear m
				for m = [7 8]
					jointData = get(MODEL.frames(m,4),'UserData');
					jointData.currentMatrix = jointData.homeMatrix*inv(Rz);
					jointData.currentAngle = -thetaGR;
					set(MODEL.frames(m,4),	'Matrix',jointData.currentMatrix,...
											'UserData',jointData)
				end; clear m
				set(MODEL.roll(1),'XData',[-current_config(6)/2-.25 current_config(6)/2+.25])
				jointData = get(MODEL.frames(4,5),'UserData');
				Rz = makehgtform('zrotate',current_settings(4)*pi/180);
				jointData.currentMatrix = Rz*jointData.homeMatrix;
				jointData.currentAngle = current_settings(4);
				set(MODEL.frames(4,5),	'Matrix',jointData.currentMatrix,...
										'UserData',jointData)
			case 'animate01'
				setappdata(GUI.fig,'AnimationInProgress',true)
				goHome()
				set(RECORD.listbox,'Value',3)
				waypoint_total = getappdata(GUI.fig,'WaypointTotal');
				waypoint_RAW = getappdata(GUI.fig,'WaypointRAW');
				waypoint_RAW = waypoint_RAW(1:waypoint_total,1:6);
				waypoint_GripRELATIVE = waypointComputeRelative(waypoint_RAW);
				waypoint_GripRELATIVE = waypoint_GripRELATIVE(:,6);	
				waypoint_GripSTEPS = waypoint_GripRELATIVE/10;
				waypoint_RAW(2:waypoint_total,6) = GRsolver(waypoint_RAW(2:waypoint_total,6)) - GRsolver(0);
				waypoint_RELATIVE = waypointComputeRelative(waypoint_RAW);
				waypoint_AnimateSTEPS = waypoint_RELATIVE/10;
				for waypoint_active = 1:waypoint_total
					setappdata(GUI.fig,'WaypointActive',waypoint_active)
					for steps = 1:10
						%% Update 3D Model
						for m = 1:5
							jointData = get(MODEL.frames(m,4),'UserData');
							Rz = makehgtform('zrotate',waypoint_AnimateSTEPS(waypoint_active,m)*pi/180);
							jointData.currentMatrix = Rz*jointData.currentMatrix;
							jointData.currentAngle = waypoint_AnimateSTEPS(waypoint_active,m);
							set(MODEL.frames(m,4),	'Matrix',jointData.currentMatrix,...
													'UserData',jointData)
						end; clear m
						thetaGR = waypoint_AnimateSTEPS(waypoint_active,6);
						Rz = makehgtform('zrotate',thetaGR*pi/180);
						for m = [6 9]
							jointData = get(MODEL.frames(m,4),'UserData');
							jointData.currentOpening = waypoint_GripSTEPS(waypoint_active);
							set(MODEL.frames(m,4),	'Matrix',get(MODEL.frames(m,4),'Matrix')*Rz,...
													'UserData',jointData)
						end; clear m
						for m = [7 8]
							jointData = get(MODEL.frames(m,4),'UserData');
							jointData.currentOpening = waypoint_GripSTEPS(waypoint_active);
							set(MODEL.frames(m,4),	'Matrix',get(MODEL.frames(m,4),'Matrix')*inv(Rz),...
													'UserData',jointData)
						end; clear m
						%% Update User Interface
						jointData = get(MODEL.frames(6,4),'UserData');
						current_settings = getappdata(GUI.fig,'CurrentSettings');
						for m = [1:3 5]
							jointData = get(MODEL.frames(m,4),'UserData');
							current_settings(m) = current_settings(m) + jointData.currentAngle;
						end; clear m
						current_settings(4) = Psolver(MODEL.frames);
						current_settings(6:8) = FKsolver(MODEL.frames,1:5);
						jointData = get(MODEL.frames(6,4),'UserData');
						current_settings(9) = current_settings(9) + jointData.currentOpening;
						setappdata(GUI.fig,'CurrentSettings',current_settings)
						set(MODEL.roll(1),'XData',[-current_settings(9)/2-.25 current_settings(9)/2+.25])
						jointData = get(MODEL.frames(4,5),'UserData');
						Rz = makehgtform('zrotate',current_settings(4)*pi/180);
						jointData.currentMatrix = Rz*jointData.homeMatrix;
						jointData.currentAngle = current_settings(4);
						set(MODEL.frames(4,5),	'Matrix',jointData.currentMatrix,...
												'UserData',jointData)
						updateTarget()
						updateSliders()
						updateTextboxes()
						testTarget()
						testCollision()
						if getappdata(GUI.fig,'Collision')
							setappdata(GUI.fig,'EnableImplementation',false)
							str = sprintf('A collision will occur from waypoint %0.0f to waypoint %0.0f. You will be unable to implement this sequence to the TeachMover. Please correct either waypoint or add an intermediate step to avoid a collision.',waypoint_active-1,waypoint_active);
							setappdata(GUI.fig,'WaypointActive',get(RECORD.listbox,'Value')-2)
							errordlg(str,'Collision Warning','modal')
							setappdata(GUI.fig,'AnimationInProgress',false)
							return
						end
						pause(0.01)
					end; clear steps
					set(RECORD.listbox,'Value',waypoint_active + 2)
					updateActiveWaypoint()
				end; clear waypoint_active
				setappdata(GUI.fig,'AnimationInProgress',false)
		end
	end

%% Helper functions -- TESTING
	function testTarget_Fcn
		current_settings = getappdata(GUI.fig,'CurrentSettings');
		endEffector_position = FKsolver(MODEL.frames,1:5);
		target_position = current_settings(6:8);
		if abs(endEffector_position - target_position) < 0.0001
			set(MODEL.target(1),'Marker','o',...
								'MarkerEdgeColor',[0 1 0],...
								'MarkerFaceColor',[0 .7 0],...
								'MarkerSize',8,...
								'LineWidth',1)
			setappdata(GUI.fig,'TargetReached',true)
		else
			set(MODEL.target(1),'Marker','x',...
								'MarkerEdgeColor',[1 1 0],...
								'MarkerFaceColor','none',...
								'MarkerSize',12,...
								'LineWidth',2)
			setappdata(GUI.fig,'TargetReached',false)
		end
	end

	function testCollision_Fcn
		trackers = zeros(9:1);
		trackers(3,:) = FKsolver(MODEL.frames,1:3);
		trackers(5,:) = FKsolver(MODEL.frames,1:5);
		trackers(6,:) = FKsolver(MODEL.frames,1:6);
		trackers(7,:) = FKsolver(MODEL.frames,[1:5 7]);
		trackers(8,:) = FKsolver(MODEL.frames,[1:5 6 8]);
		trackers(9,:) = FKsolver(MODEL.frames,[1:5 7 9]);
		radius = trackers(:,1) + i*trackers(:,2);
		radius = abs(radius);
		if any(trackers([3 6:9],3) < 0.2) || trackers(5,3) < -0.01 ||(any(radius([3 5:9]) < 0.55) && any(trackers([3 5:9],3) < 2.2))
			set(GUI.warnings,'Visible','on')
			setappdata(GUI.fig,'Collision',true)
			set(MODEL.target(1),'Marker','x',...
								'MarkerEdgeColor',[1 0 0],...
								'MarkerFaceColor','none',...
								'MarkerSize',50,...
								'LineWidth',20)
			set(RECORD.setWaypoint,'Enable','off')
			set(IMPLEMENT.onlineSingle,'Enable','off')
			set(IMPLEMENT.onlineAll,'Enable','off')
			set(IMPLEMENT.offline,'Enable','off')
		elseif click && (any(trackers([3 6:9],3) < 0.2) || trackers(5,3) < -0.01 || (any(radius([3 5:9]) < 0.55)) && any(trackers([3 5:9],3) < 2.2))
			set(GUI.warnings,'Visible','off')
			setappdata(GUI.fig,'Collision',false)
			set(RECORD.setWaypoint,'Enable','on')
			set(IMPLEMENT.onlineSingle,'Enable','on')
			set(IMPLEMENT.onlineAll,'Enable','on')
			set(IMPLEMENT.offline,'Enable','on')
		else
			set(GUI.warnings,'Visible','off')
			setappdata(GUI.fig,'Collision',false)
			set(RECORD.setWaypoint,'Enable','on')
			set(IMPLEMENT.onlineSingle,'Enable','on')
			set(IMPLEMENT.onlineAll,'Enable','on')
			set(IMPLEMENT.offline,'Enable','on')
		end
	end

%% Helper functions -- RECORDING
	function responseAddWaypoint_Fcn(object,eventdata)
		updateApplicationData('add waypoint')
		updateActiveWaypoint()
		updateTotalWaypoint()
		updateListbox()
	end

	function responseReduceWaypoint_Fcn(object,eventdata)
		updateApplicationData('reduce waypoint')
		updateActiveWaypoint()
		updateTotalWaypoint()
		updateListbox()
	end

	function responseListbox_Fcn(object,eventdata)
		if ~getappdata(GUI.fig,'AnimationInProgress')
			updateApplicationData('listbox')
			updateModel('IK direct')
			updateSliders()
			updateTextboxes()
			updateTarget()
			updateActiveWaypoint()
			testTarget()
			testCollision()
		end
	end

	function responseSetWaypoint_Fcn(object,eventdata)
		updateApplicationData('set waypoint')
		updateListbox()
	end

	function responseNewSequence_Fcn(object,eventdata)
		setappdata(GUI.fig,'WaypointActive',1)
		setappdata(GUI.fig,'WaypointTotal',1)
		setappdata(GUI.fig,'WaypointRAW',home_listed)
		setappdata(GUI.fig,'WaypointRELATIVE',home_relative)
		goHome()
		updateListbox()
		updateActiveWaypoint()
		updateTotalWaypoint()
	end

	function responseLoadSequence_Fcn(object,eventdata)
		sequence_Filename = uigetfile('*.mat','Load TeachMover waypoint sequence');
		if ischar(sequence_Filename)
			responseNewSequence(RECORD.newSequence,[])
			load(sequence_Filename,'waypoint_RAW');
		else
			return
		end
		if exist('waypoint_RAW')
			setappdata(GUI.fig,'WaypointActive',1)
			setappdata(GUI.fig,'WaypointTotal',size(waypoint_RAW,1))
			waypoint_total = getappdata(GUI.fig,'WaypointTotal');
			additional = repmat(home_listed_small,50-waypoint_total,1);
			waypoint_RAW = cat(1,waypoint_RAW,additional);
			setappdata(GUI.fig,'WaypointRAW',waypoint_RAW)
			waypoint_RAW = waypoint_RAW(1:size(waypoint_RAW,1),1:6);
			waypoint_RELATIVE = waypointComputeRelative(waypoint_RAW);
			setappdata(GUI.fig,'WaypointRELATIVE',waypoint_RELATIVE)
			goHome()
			updateListbox()
			updateActiveWaypoint()
			updateTotalWaypoint()
		else
			errordlg('The contents of the file you selected cannot be parsed by TeachMover Virtual Command. Select a MAT file which contains a TeachMover Waypoint Sequence.','Invalid file contents','modal')
			return
		end
	end

	function responseSaveSequence_Fcn(object,eventdata)
		waypoint_total = getappdata(GUI.fig,'WaypointTotal');
		waypoint_RAW = getappdata(GUI.fig,'WaypointRAW');
		waypoint_RAW = waypoint_RAW(1:waypoint_total,:);
		sequence_Filename = uiputfile('*.mat','Save TeachMover waypoint sequence');
		save(sequence_Filename,'waypoint_RAW')
	end

	function updateActiveWaypoint_Fcn
		waypoint_active = getappdata(GUI.fig,'WaypointActive');
		waypoint_total = getappdata(GUI.fig,'WaypointTotal');
		if waypoint_total < waypoint_active
			waypoint_active = waypoint_total;
			setappdata(GUI.fig,'WaypointActive',waypoint_active);
		end
		set(RECORD.activeWaypoint,'String',num2str(waypoint_active,'%02.0f'))
	end

	function updateTotalWaypoint_Fcn
		waypoint_total = getappdata(GUI.fig,'WaypointTotal');
		set(RECORD.totalWaypoint,'String',num2str(waypoint_total,'%02.0f'))
	end

	function updateListbox_Fcn
		waypoint_active = getappdata(GUI.fig,'WaypointActive');
		waypoint_total = getappdata(GUI.fig,'WaypointTotal');
		if waypoint_total == waypoint_active
			set(RECORD.listbox,'Value',3)
		end
		waypoint_RAW = getappdata(GUI.fig,'WaypointRAW');
		waypoint_RAW = waypoint_RAW(1:waypoint_total,:);
		waypoint_list{9} = [];
		for m = 1:9
			waypoint_list{m} = cellstr(num2str(waypoint_RAW(:,m),'%0.1f'));
		end; clear m
		A = waypoint_list;
		formattable(RECORD.listbox,{'  TR' ' SH' '  EL' '  P' '  R' 'GR' ' x' ' y' ' z'},A{1:9})
		set(RECORD.listbox,'Value',waypoint_active + 2)
	end

%% Helper functions -- PLAYBACK
	function responseAnimate_Fcn(object,eventdata)
		if ~getappdata(GUI.fig,'AnimationInProgress')
			updateModel('animate01')
		end
	end

%% Helper functions -- SPECIAL
	function goHome_Fcn(object,evendata)
		setappdata(GUI.fig,'CurrentSettings',home_settings)
		setappdata(GUI.fig,'CurrentConfigurationAngleSIM',zeros(1,6))
		updateApplicationData('IK direct')
		updateModel('IK')
		updateTarget()
		updateSliders()
		updateTextboxes()
		testTarget()
		testCollision()
	end

	function initializeListbox_Fcn
		waypoint_total = getappdata(GUI.fig,'WaypointTotal');
		waypoint_RAW = getappdata(GUI.fig,'WaypointRAW');
		waypoint_RAW = waypoint_RAW(1:waypoint_total,:);
		waypoint_list{9} = [];
		for m = 1:9
			waypoint_list{m} = cellstr(num2str(waypoint_RAW(:,m),'%0.1f'));
		end; clear m
		A = waypoint_list;
		formattable(RECORD.listbox,{' TR' ' SH' '  EL' '  P' ' R' 'GR' ' x' ' y' ' z'},A{1},A{2},A{3},A{4},A{5},A{6},A{7},A{8},A{9})
		set(RECORD.listbox,'Value',3)
		updateTotalWaypoint()
	end

	function makeSimLikeActual_Fcn(TM_config_angle)
		TM_config_angle(4) = sum(TM_config_angle(2:4));
		TM_config_settings = [TM_config_angle(1:5) FKsolver_RAW(TM_config_angle) TM_config_angle(6)];
		setappdata(GUI.fig,'CurrentSettings',TM_config_settings)
		setappdata(GUI.fig,'CurrentConfigurationAngleSIM',zeros(1,6))
		updateApplicationData('IK direct')
		updateModel('IK')
		updateTarget()
		updateSliders()
		updateTextboxes()
		testTarget()
		testCollision()
	end

%% Helper functions -- IMPLEMENTATION
	function ONLINE_Single_Fcn(object,eventdata)
		clc
		desired_config = getappdata(GUI.fig,'CurrentConfigurationAngleSIM');
		% Read actual status of TeachMover
			serialPort = openPort();
			[TM_config_steps TM_config_angle] = TM_read(serialPort);
			closePort(serialPort);
		% Make model imitate actual
		makeSimLikeActual(TM_config_angle);
		AnimateRELATIVE = desired_config - TM_config_angle;
		AnimateSTEPS = AnimateRELATIVE/10;
%%-------------------------------------------------------------------------
		if ~getappdata(GUI.fig,'AnimationInProgress')
			setappdata(GUI.fig,'AnimationInProgress',true)
			disp('Testing for collisions...')
			for steps = 1:10
				%% Update 3D Model
				for m = 1:5
					jointData = get(MODEL.frames(m,4),'UserData');
					Rz = makehgtform('zrotate',AnimateSTEPS(m)*pi/180);
					jointData.currentMatrix = Rz*jointData.currentMatrix;
					jointData.currentAngle = AnimateSTEPS(m);
					set(MODEL.frames(m,4),	'Matrix',jointData.currentMatrix,...
											'UserData',jointData)
				end; clear m
				thetaGR = GRsolver(AnimateSTEPS(6)) - GRsolver(0);
				Rz = makehgtform('zrotate',thetaGR*pi/180);
				for m = [6 9]
					jointData = get(MODEL.frames(m,4),'UserData');
					jointData.currentOpening = AnimateSTEPS(6);
					set(MODEL.frames(m,4),	'Matrix',get(MODEL.frames(m,4),'Matrix')*Rz,...
											'UserData',jointData)
				end; clear m
				for m = [7 8]
					jointData = get(MODEL.frames(m,4),'UserData');
					jointData.currentOpening = AnimateSTEPS(6);
					set(MODEL.frames(m,4),	'Matrix',get(MODEL.frames(m,4),'Matrix')*inv(Rz),...
											'UserData',jointData)
				end; clear m
				%% Update User Interface
				current_settings = getappdata(GUI.fig,'CurrentSettings');
				for m = [1:3 5]
					jointData = get(MODEL.frames(m,4),'UserData');
					current_settings(m) = current_settings(m) + jointData.currentAngle;
				end; clear m
				current_settings(4) = Psolver(MODEL.frames);
				current_settings(6:8) = FKsolver(MODEL.frames,1:5);
				jointData = get(MODEL.frames(6,4),'UserData');
				current_settings(9) = current_settings(9) + jointData.currentOpening;
				setappdata(GUI.fig,'CurrentSettings',current_settings)
				set(MODEL.roll(1),'XData',[-current_settings(9)/2-.25 current_settings(9)/2+.25])
				jointData = get(MODEL.frames(4,5),'UserData');
				Rz = makehgtform('zrotate',current_settings(4)*pi/180);
				jointData.currentMatrix = Rz*jointData.homeMatrix;
				jointData.currentAngle = current_settings(4);
				set(MODEL.frames(4,5),	'Matrix',jointData.currentMatrix,...
										'UserData',jointData)
				updateTarget()
				updateSliders()
				updateTextboxes()
				testTarget()
				testCollision()
				if getappdata(GUI.fig,'Collision')
					setappdata(GUI.fig,'EnableImplementation',false)
					str = sprintf('A collision will occur. You will be unable to implement this sequence to the TeachMover. Please make corrections or add an intermediate step to avoid a collision.');
					setappdata(GUI.fig,'WaypointActive',get(RECORD.listbox,'Value')-2)
					errordlg(str,'Collision Warning','modal')
					setappdata(GUI.fig,'AnimationInProgress',false)
					return
				end
				pause(0.01)
			end; clear steps
%%-------------------------------------------------------------------------
			setappdata(GUI.fig,'AnimationInProgress',false);
			setappdata(GUI.fig,'CurrentConfigurationAngleSIM',desired_config)
			currentConfigurationAngleSIM = getappdata(GUI.fig,'CurrentConfigurationAngleSIM');
		end
		clc
		disp('Bringing TeachMover to desired position as shown on screen...')
		status = TM_ONLINE_Single(desired_config,TM_config_steps,openPort,closePort);
	end

	function ONLINE_All_Fcn(object,eventdata)
		clc
		goHome();
		drawnow
		home_config = getappdata(GUI.fig,'CurrentConfigurationAngleSIM');
		serialPort = openPort();
		[TM_config_steps TM_config_angle] = TM_read(serialPort);
		closePort(serialPort);
		disp('Returning TeachMover to home position...')
		status = TM_ONLINE_Single(home_config,TM_config_steps,openPort,closePort);
		disp('Testing for collisions...')
		updateModel('animate01')
		clc
		if ~getappdata(GUI.fig,'Collision')
			disp('No collisions detected. Relay command to TeachMover.')
			waypoint_total			= getappdata(GUI.fig,'WaypointTotal');
			waypoint_RAW			= getappdata(GUI.fig,'WaypointRAW');
			waypoint_RAW			= waypoint_RAW(1:waypoint_total,1:6);
			waypoint_RELATIVE		= waypointComputeRelative(waypoint_RAW);
			status = TM_ONLINE_All(waypoint_RELATIVE,waypoint_total,openPort,closePort);
		else
			disp('Collision! No go.')
		end
	end
	
	function OFFLINE_Fcn(object,eventdata)
		clc
		goHome();
		drawnow
		home_config = getappdata(GUI.fig,'CurrentConfigurationAngleSIM');
		serialPort = openPort();
		[TM_config_steps TM_config_angle] = TM_read(serialPort);
		closePort(serialPort);
		disp('Returning TeachMover to home position...')
		status = TM_ONLINE_Single(home_config,TM_config_steps,openPort,closePort);
		disp('Testing for collisions...')
		updateModel('animate01')
		clc
		if ~getappdata(GUI.fig,'Collision')
			disp('No collisions detected. Relay command to TeachMover.')
			waypoint_total			= getappdata(GUI.fig,'WaypointTotal');
			waypoint_RAW			= getappdata(GUI.fig,'WaypointRAW');
			waypoint_RAW			= waypoint_RAW(1:waypoint_total,1:6);
			waypoint_RAW_STEPS		= convertToSTEPS(waypoint_RAW);
			serialPort = openPort();
			for stepNumber = 1:waypoint_total
				commandString = convertToQWRITE(stepNumber,240,waypoint_RAW_STEPS(stepNumber,:));
				disp(commandString);
				fprintf(serialPort,commandString);
				statusPort = fgetl(serialPort);
			end; clear stepNumber
			closePort(serialPort)
		else
			disp('Collision! No go.')
		end
	end

	function [TM_config_steps TM_config_angle] = TM_read(serialPort)
		command = sprintf('@READ');
		fprintf(serialPort,command);
		response1 = fgetl(serialPort);
		TM_config_steps = fgetl(serialPort);
		TM_config_steps = str2num(TM_config_steps);
		TM_config_steps = TM_config_steps(1:6);
 		%	Calculate actual angle configuration of TM based on step values
		%	read from serial port
		TR = TM_config_steps(1)/19.64;
		SH = -TM_config_steps(2)/19.64;
		EL = TM_config_steps(2)/19.64 - TM_config_steps(3)/11.55;
		GR = (TM_config_steps(6) - TM_config_steps(3))/371;
		Ro = (TM_config_steps(5) - TM_config_steps(4))/2/4.27;
		P  = TM_config_steps(3)/11.55 - (TM_config_steps(5) + TM_config_steps(4))/2/4.27;
		TM_config_angle = [TR SH EL P Ro GR];
	end

	function serialPort = openPort_Fcn
		s = instrfind('Status','open');
		if ~isempty(s)
			fclose(s)
			delete(s)
		end
		clear s
		serialPort = serial('COM1',...
							'Terminator','CR',...
							'DataTerminalReady','off',...
							'RequestToSend','off',...
							'Timeout',2);
		fopen(serialPort); fprintf(serialPort,'@READ')
		testRead = fgetl(serialPort); testRead = fgetl(serialPort);
		if isempty(testRead)
			errordlg('The TeachMover is not connected to COM1','TeachMover not detected','modal')
			fclose(serialPort)
		else
			set(serialPort,'Timeout',60)
		end
	end

	function closePort_Fcn(serialPort)
		fclose(serialPort);
		delete(serialPort);
	end

%% Graphics fix
	for m = 1:2
		set(GUI.fig,'WVisual','50')
		drawnow
	end; clear m
	pack
	drawnow
end