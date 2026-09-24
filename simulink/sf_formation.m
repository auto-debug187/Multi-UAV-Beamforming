function sf_formation(block)
% Sampled centralized formation coordinator. One physical state vector per UAV.
block.NumDialogPrms=1; cfg=block.DialogPrm(1).Data;
block.NumInputPorts=1; block.NumOutputPorts=1;
block.SetPreCompInpPortInfoToDynamic; block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=17*cfg.N;
block.InputPort(1).DirectFeedthrough=true;
block.OutputPort(1).Dimensions=3*cfg.N;
block.InputPort(1).SamplingMode='Sample'; block.OutputPort(1).SamplingMode='Sample';
block.InputPort(1).DatatypeID=0; block.OutputPort(1).DatatypeID=0;
block.InputPort(1).Complexity='Real'; block.OutputPort(1).Complexity='Real';
block.SampleTimes=[cfg.Ts 0];
block.DialogPrmsTunable={'Nontunable'};
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('Outputs',@output);
end
function output(block)
cfg=block.DialogPrm(1).Data;
X=reshape(block.InputPort(1).Data,17,cfg.N);
a=bf_formationControl(block.CurrentTime,X,cfg);
block.OutputPort(1).Data=a(:);
end
