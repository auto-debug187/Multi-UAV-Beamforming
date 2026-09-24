function sf_autopilot(block)
block.NumDialogPrms=1; cfg=block.DialogPrm(1).Data;
block.NumInputPorts=2; block.NumOutputPorts=2;
block.SetPreCompInpPortInfoToDynamic; block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=3; block.InputPort(2).Dimensions=17;
for k=1:2
    block.InputPort(k).DirectFeedthrough=true;
    block.InputPort(k).SamplingMode='Sample';
    block.OutputPort(k).Dimensions=4;
    block.OutputPort(k).SamplingMode='Sample';
    block.InputPort(k).DatatypeID=0; block.OutputPort(k).DatatypeID=0;
    block.InputPort(k).Complexity='Real'; block.OutputPort(k).Complexity='Real';
end
block.SampleTimes=[cfg.Ts 0]; block.DialogPrmsTunable={'Nontunable'};
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('Outputs',@output);
end
function output(block)
[u,d]=bf_autopilot(block.InputPort(1).Data,block.InputPort(2).Data,block.DialogPrm(1).Data);
block.OutputPort(1).Data=u; block.OutputPort(2).Data=d;
end
