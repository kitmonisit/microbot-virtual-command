function status = TM_ONLINE_Single(current_config,TM_config_steps,openPort,closePort)
%	This function will implement to the TeachMover the position shown on
%	screen.
	stepsAbsolute = convertToSTEPS(current_config);
	serialPort = openPort();
	closePort(serialPort);
	stepsRelative = stepsAbsolute - TM_config_steps;
	serialPort = openPort();
	status = TM_command(serialPort,stepsRelative);
	closePort(serialPort);
end

function statusPort = TM_command(serialPort,stepsRelative)
	command = sprintf('@STEP 240,%0.0f,%0.0f,%0.0f,%0.0f,%0.0f,%0.0f',stepsRelative);
	disp(command)
	fprintf(serialPort,command);
	statusPort = fgetl(serialPort);
end