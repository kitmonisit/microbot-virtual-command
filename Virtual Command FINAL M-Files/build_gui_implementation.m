function IMPLEMENT = build_gui_implementation(fig)
	GUI.fig = fig;
	clear fig

	IMAGES.current = imread('IMAGES -- Current.jpg');
	IMAGES.all = imread('IMAGES -- All.jpg');
	IMAGES.offline = imread('IMAGES -- Offline.jpg');
	IMPLEMENT.onlineSingle	= uicontrol(...
					'Units','pixels',...
					'Parent',GUI.fig,...
					'Position',[932 412 64 64],...
					'Interruptible','off',...
					'CData',IMAGES.current,...
					'BusyAction','cancel',...
					'TooltipString','Implement current position to the TeachMover (ONLINE)');
	IMPLEMENT.onlineAll		= uicontrol(...
					'Units','pixels',...
					'Parent',GUI.fig,...
					'Position',[1000 412 64 64],...
					'Interruptible','off',...
					'CData',IMAGES.all,...
					'BusyAction','cancel',...
					'TooltipString','Implement entire sequence to the TeachMover (ONLINE)');
	IMPLEMENT.offline		= uicontrol(...
					'Units','pixels',...
					'Parent',GUI.fig,...
					'Position',[1068 412 64 64],...
					'Interruptible','off',...
					'CData',IMAGES.offline,...
					'BusyAction','cancel',...
					'TooltipString','Record entire sequence to TeachMover memory (OFFLINE)');
end