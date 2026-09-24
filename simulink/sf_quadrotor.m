function sf_quadrotor(block)
% Continuous 17-state plant; integrated by the Simulink ode4 solver.
block.NumDialogPrms=2;
block.NumInputPorts=1; block.NumOutputPorts=1;
block.SetPreCompInpPortInfoToDynamic; block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=4; block.InputPort(1).DirectFeedthrough=false;
block.OutputPort(1).Dimensions=17;
block.InputPort(1).SamplingMode='Sample'; block.OutputPort(1).SamplingMode='Sample';
block.InputPort(1).DatatypeID=0; block.OutputPort(1).DatatypeID=0;
block.InputPort(1).Complexity='Real'; block.OutputPort(1).Complexity='Real';
block.NumContStates=17; block.SampleTimes=[0 0];
block.DialogPrmsTunable={'Nontunable','Nontunable'};
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('InitializeConditions',@initialize);
block.RegBlockMethod('Outputs',@output);
block.RegBlockMethod('Derivatives',@derivatives);
end
function initialize(block)
cfg=block.DialogPrm(1).Data; i=block.DialogPrm(2).Data;
block.ContStates.Data=cfg.x0(:,i);
end
function output(block)
block.OutputPort(1).Data=block.ContStates.Data;
end
function derivatives(block)
block.Derivatives.Data=bf_plantRHS(block.CurrentTime,block.ContStates.Data, ...
    block.InputPort(1).Data,block.DialogPrm(1).Data);
end
