function status = TM_ONLINE_All(waypoint_RELATIVE,waypoint_total,openPort,closePort)
%	This function will implement to the TeachMover the entire motion
%	sequence shown on the listbox.
	stepsRelative = convertToSTEPS(waypoint_RELATIVE);
	ProgressBar = waitbar(0);
	set(ProgressBar,	'Name','Relaying commands to TeachMover...')
	drawnow
	ProgressBarAxes = get(ProgressBar,'Children');
	ProgressBarTitle = get(ProgressBarAxes,'Title'); clear ProgressBarAxes;
	serialPort = openPort();
	status = '1';
	for m = 1:waypoint_total
		if status == '1'
			waitbar(m/waypoint_total,ProgressBar)
			status = TM_command(serialPort,stepsRelative(m,:),ProgressBarTitle);
		else
			set(ProgressBarTitle,'String','Action canceled using teach pendant.')
			pause(0.1)
		end
	end; clear m
	close(ProgressBar)
	closePort(serialPort);
end

function statusPort = TM_command(serialPort,stepsRelative,ProgressBarTitle)
	command = sprintf('@STEP 240,%0.0f,%0.0f,%0.0f,%0.0f,%0.0f,%0.0f',stepsRelative);
	disp(command)
	set(ProgressBarTitle,'String',command)
	drawnow
	fprintf(serialPort,command);
	statusPort = fgetl(serialPort);
end
