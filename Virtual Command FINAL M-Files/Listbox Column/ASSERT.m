function ASSERT(cond,msg)
%ASSERT - Similar to the C "assert" function
%
% ASSERT(cond,msg)
%
% cond should be an expression which is believed to be true.
% msg is a message describing the assertion.
%
% e.g. ASSERT(x>5 && x<20,'x is within the allowed limits')
%
% This should not be thought of as "normal" error handling.
% It is designed to help catch bugs in the code, and so the
% call stack is printed in the command window (so that it
% is not lost if the error is caught).

if ~cond
    fprintf(1,'Assertion Failure: %s\n',msg);
    dbstack;
    error('MATLAB:Utilities:AssertionFailure','Assertion Failure: %s',msg);
end

