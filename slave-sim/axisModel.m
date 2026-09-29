function [ax, G] = axisModel(ax, G, E, sim, hook, dt)
%AXISMODEL Stand-in for the LD POU, the motion function blocks it owns, the
%   SERVOPACK and the mechanics.
%
%   Reads the G_cmd* structs written by PRG_Main and writes the G_sta*
%   structs and G_sysActual*. The axis follows its command exactly (no
%   servo dynamics, no load) unless hook.blocked holds it still.
%
%   Modelled behaviour
%     MC_Power         Status follows Enable after sim.powerOnDelay.
%     MC_Reset         Done the scan after Execute rises, held while Execute.
%     MC_Stop          Decelerates at Deceleration; Busy while moving, Done at
%                      rest while Execute. Aborts MC_MoveAbsolute, Jog and
%                      Y_DirectControl.
%     Y_DirectControl  PositionMode passes Position straight through;
%                      VelocityTLMode slews to Velocity at Acceleration /
%                      Deceleration; TorqueVLMode accelerates in proportion
%                      to Torque up to the Velocity limit.
%     MC_MoveAbsolute  Rising Execute starts a trapezoidal move. Done is held
%                      while Execute, or for one scan if Execute has dropped.
%     Jog              Level-sensitive Forward / Reverse; Done pulses for one
%                      scan once the axis has stopped after both are FALSE.
%     MC_SetPosition   Rising Execute shifts the controller frame so that the
%                      present position reads Position.
%   Priority: MC_Stop, Y_DirectControl, MC_MoveAbsolute, Jog.
%
%   ax.x is the physical position and ax.offset the controller frame shift:
%   G_sysActualPosition = ax.x + ax.offset.

% --- MC_Power ---------------------------------------------------------------
if G.G_cmdPower.Enable
    ax.tPower = min(ax.tPower + dt, sim.powerOnDelay);
else
    ax.tPower = 0;
end
powered = ax.tPower >= sim.powerOnDelay;
G.G_staPower.Status = powered;
G.G_staPower.Error = false;

% --- MC_Reset ---------------------------------------------------------------
G.G_staReset.Done = G.G_cmdReset.Execute && ax.prevResetExec;
G.G_staReset.Busy = false;
G.G_staReset.Error = false;
ax.prevResetExec = G.G_cmdReset.Execute;

% --- MC_SetPosition ---------------------------------------------------------
if G.G_cmdSetPosition.Execute && ~ax.prevSetPosExec
    if G.G_cmdSetPosition.Mode
        ax.offset = ax.offset + G.G_cmdSetPosition.Position;
    else
        ax.offset = G.G_cmdSetPosition.Position - ax.x;
    end
end
G.G_staSetPosition.Done = G.G_cmdSetPosition.Execute;
G.G_staSetPosition.Busy = false;
G.G_staSetPosition.Error = false;
ax.prevSetPosExec = G.G_cmdSetPosition.Execute;

% --- MC_MoveAbsolute: start on rising Execute --------------------------------
moveExec = G.G_cmdMoveAbsolute.Execute;
if moveExec && ~ax.prevMoveExec && powered && ~G.G_cmdStop.Execute
    ax.moveActive = true;
    ax.moveDone = false;
    ax.moveAborted = false;
    ax.moveTarget = G.G_cmdMoveAbsolute.Position - ax.offset;
    ax.moveVel = abs(G.G_cmdMoveAbsolute.Velocity);
    ax.moveAcc = G.G_cmdMoveAbsolute.Acceleration;
    ax.moveDec = G.G_cmdMoveAbsolute.Deceleration;
end
moveDoneOut = ax.moveDone;
if ax.moveDone && ~moveExec
    ax.moveDone = false;
end
if ~moveExec
    ax.moveAborted = false;
end
ax.prevMoveExec = moveExec;

% --- Motion arbitration -----------------------------------------------------
stopExec = G.G_cmdStop.Execute;
dcEnable = G.G_cmdDirectControl.Enable;
jogCmd = G.G_cmdJog.Forward || G.G_cmdJog.Reverse;
jogDone = false;
dcActive = false;
xDirect = ax.x;
directPosition = false;

if ~powered
    ax.v = 0;
    ax.moveActive = false;
    ax.jogBusy = false;

elseif stopExec
    if ax.moveActive
        ax.moveActive = false;
        ax.moveAborted = true;
    end
    ax.jogBusy = false;
    ax.v = slew(ax.v, 0, G.G_cmdStop.Deceleration * dt);

elseif dcEnable
    dcActive = true;
    if ax.moveActive
        ax.moveActive = false;
        ax.moveAborted = true;
    end
    ax.jogBusy = false;
    mode = G.G_cmdDirectControl.ControlMode;
    if mode == E.Y_ControlMode.PositionMode
        directPosition = true;
        xDirect = G.G_cmdDirectControl.Position - ax.offset;
    elseif mode == E.Y_ControlMode.VelocityTLMode
        vT = G.G_cmdDirectControl.Velocity;
        if abs(vT) > abs(ax.v)
            rate = G.G_cmdDirectControl.Acceleration;
        else
            rate = G.G_cmdDirectControl.Deceleration;
        end
        ax.v = slew(ax.v, vT, rate * dt);
    else
        vLim = abs(G.G_cmdDirectControl.Velocity);
        ax.v = ax.v + G.G_cmdDirectControl.Torque * sim.torqueAccel * dt;
        ax.v = min(max(ax.v, -vLim), vLim);
    end

elseif ax.moveActive
    dist = ax.moveTarget - ax.x;
    aDt = ax.moveDec * dt;
    rateStop = -aDt + sqrt(aDt * aDt + 2 * ax.moveDec * abs(dist));
    vT = sign(dist) * min(ax.moveVel, rateStop);
    ax.v = slew(ax.v, vT, ax.moveAcc * dt);
    if abs(dist) <= abs(ax.v) * dt + 1e-9 && abs(ax.v) <= aDt + 1e-9
        directPosition = true;
        xDirect = ax.moveTarget;
        ax.moveActive = false;
        ax.moveDone = true;
        moveDoneOut = moveExec;
    end

elseif jogCmd || ax.jogBusy
    if G.G_cmdJog.Forward && ~G.G_cmdJog.Reverse
        ax.v = slew(ax.v, abs(G.G_cmdJog.Velocity), G.G_cmdJog.Acceleration * dt);
        ax.jogBusy = true;
    elseif G.G_cmdJog.Reverse && ~G.G_cmdJog.Forward
        ax.v = slew(ax.v, -abs(G.G_cmdJog.Velocity), G.G_cmdJog.Acceleration * dt);
        ax.jogBusy = true;
    else
        ax.v = slew(ax.v, 0, G.G_cmdJog.Deceleration * dt);
        if ax.v == 0
            ax.jogBusy = false;
            jogDone = true;
        end
    end

else
    ax.v = 0;
end

% --- Integrate, mechanical stall and hard stops -----------------------------
xPrev = ax.x;
if hook.blocked
    ax.v = 0;
elseif directPosition
    ax.x = xDirect;
else
    ax.x = ax.x + ax.v * dt;
end
xMin = sim.switchNegPos - sim.hardStopMargin;
xMax = sim.switchPosPos + sim.hardStopMargin;
if ax.x < xMin || ax.x > xMax
    ax.x = min(max(ax.x, xMin), xMax);
    ax.v = 0;
end
if directPosition
    ax.v = (ax.x - xPrev) / dt;
end

% --- Status -----------------------------------------------------------------
G.G_staStop.Busy = stopExec && ax.v ~= 0;
G.G_staStop.Done = stopExec && ax.v == 0 && ax.prevStopExec;
G.G_staStop.Error = false;
ax.prevStopExec = stopExec;

G.G_staDirectControl.Active = dcActive;
G.G_staDirectControl.Error = false;

G.G_staMoveAbsolute.Busy = ax.moveActive;
G.G_staMoveAbsolute.Active = ax.moveActive;
G.G_staMoveAbsolute.Done = moveDoneOut;
G.G_staMoveAbsolute.CommandAborted = ax.moveAborted;
G.G_staMoveAbsolute.Error = false;

G.G_staJog.Busy = ax.jogBusy;
G.G_staJog.InVelocity = ax.jogBusy && jogCmd && abs(ax.v) == abs(G.G_cmdJog.Velocity);
G.G_staJog.Done = jogDone;
G.G_staJog.Error = false;

G.G_sysActualPosition = ax.x + ax.offset;
G.G_sysActualVelocity = ax.v;
G.G_sysActualTorque = hook.torque;
G.G_sysDriveFault = hook.driveFault;
end


function v = slew(v, target, step)
% Move v toward target by at most step.
v = v + min(max(target - v, -step), step);
end
