function sf_liveview(block)
block.NumDialogPrms=1; cfg=block.DialogPrm(1).Data;
block.NumInputPorts=1; block.NumOutputPorts=0;
block.SetPreCompInpPortInfoToDynamic;
block.InputPort(1).Dimensions=17*cfg.N;
block.InputPort(1).SamplingMode='Sample';
block.InputPort(1).DirectFeedthrough=true;
block.InputPort(1).DatatypeID=0; block.InputPort(1).Complexity='Real';
block.SampleTimes=[cfg.visualTs 0]; block.DialogPrmsTunable={'Nontunable'};
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('Start',@start);
block.RegBlockMethod('Outputs',@output);
end
function start(block)
h=bf_live_init(block.DialogPrm(1).Data);
set_param(block.BlockHandle,'UserData',h,'UserDataPersistent','off');
end
function output(block)
h=get_param(block.BlockHandle,'UserData');
if isempty(h) || ~isgraphics(h.fig), return; end
last=getappdata(h.fig,'lastTime');
if ~isempty(last) && block.CurrentTime<=last+1e-9, return; end
cfg=block.DialogPrm(1).Data;
bf_live_update(h,block.CurrentTime,reshape(block.InputPort(1).Data,17,cfg.N),cfg);
setappdata(h.fig,'lastTime',block.CurrentTime);
end
