function commandString = convertToQWRITE(stepNumber,speed,X)
% 	X is a 6-element array of absolute motor half-steps which will be fed to the
% 	TeachMover. They have to be converted to certain values as required by
% 	the syntax of the TeachMover's @QWRITE serial command. Note that the
% 	@QWRITE command relies on absolute values (NOT relative), as opposed to
% 	the @STEP command.

%	Make sure that X has 6 elements; pad with zeros
	if size(X,2) < 6
		X(1,(size(X,2)+1):6) = zeros;
	end

%	Break down each element of X into high- and low-order byte parts
	H = zeros(6,1);
	L = zeros(6,1);
	for m = 1:6
		% Account for negative numbers and express them in their negative
		% binary form using two's complement
		if X(m) < 0
			X(m) = bitcmp(uint16(-X(m)),16) + 1;
		end
	end; clear m
%	Convert all elements of X into unsigned 16-bit integers
	X = uint16(X);
	for m = 1:6
		% Find low-order byte
		L(m) = rem(X(m),256);
		% Find high-order byte
		H(m) = (X(m) - L(m))/256;
	end; clear m
	

%	Formulate the QWRITE argument list
	QWRITE_prelim	   = zeros(2,8);
	QWRITE_prelim(:,1) = [0; stepNumber];
	QWRITE_prelim(:,2) = [255-speed; 1];
	QWRITE_prelim(:,3) = [L(2); L(1)];
	QWRITE_prelim(:,4) = [L(4); L(3)];
	QWRITE_prelim(:,5) = [L(6); L(5)];
	QWRITE_prelim(:,6) = [H(2); H(1)];
	QWRITE_prelim(:,7) = [H(4); H(3)];
	QWRITE_prelim(:,8) = [H(6); H(5)];
	QWRITE_prelim(1,:) = QWRITE_prelim(1,:)*256;
	QWRITE_final	   = sum(QWRITE_prelim);

%	Formulate the command string
	commandString = sprintf('@QWRITE %0.0f,%0.0f,%0.0f,%0.0f,%0.0f,%0.0f,%0.0f,%0.0f',QWRITE_final);
end