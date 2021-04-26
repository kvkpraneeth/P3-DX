function sysCall_init() 
    
    robotHandle=sim.getObjectHandle(sim.handle_self)

    leftMotor=sim.getObjectHandle("Pioneer_p3dx_leftMotor")

    rightMotor=sim.getObjectHandle("Pioneer_p3dx_rightMotor")


    if simROS2 then
        
        sim.addLog(sim.verbosity_scriptinfos, "ROS2 interface was found.")
        
        leftMotorTopicName='leftMotorSpeed'
        rightMotorTopicName='rightMotorSpeed'
        
        simTimeTopicName='simTime'
        stateTopicName='pioneerState'
        
        simTimePub=simROS2.createPublisher('/'..simTimeTopicName, 'std_msgs/msg/Float32')
        statePub=simROS2.createPublisher('/'..stateTopicName, 'geometry_msgs/msg/TransformStamped')
        
        leftMotorSub=simROS2.createSubscription('/'..leftMotorTopicName, 'std_msgs/msg/Float32', 'setLeftMotorSpeed')
        rightMotorSub=simROS2.createSubscription('/'..rightMotorTopicName, 'std_msgs/msg/Float32', 'setRightMotorSpeed')
        
        result=sim.launchExecutable('pioneer',leftMotorTopicName.." "..rightMotorTopicName.." "..simTimeTopicName,0)
    else
       
       sim.addLog(sim.verbosity_scriptinfos, "ROS2 Interface was not found. Cannot run. ")
    
    end

end

function getTransformStamped(objHandle,name,relTo,relToName)

    t=simROS2.getSystemTime()
    p=sim.getObjectPosition(objHandle,relTo)
    o=sim.getObjectQuaternion(objHandle,relTo)
    return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
end

function setLeftMotorSpeed(msg)
    sim.setJointTargetVelocity(leftMotor,msg.data)
end

function setRightMotorSpeed(msg)
    sim.setJointTargetVelocity(rightMotor,msg.data)
end


function sysCall_cleanup() 
    if simROS2 then
        simROS2.shutdownSubscription(leftMotorSub)
        simROS2.shutdownSubscription(rightMotorSub)
    end
end 


function sysCall_actuation()    

    if simROS2 then
        simROS2.publish(simTimePub,{data=sim.getSimulationTime()})
        simROS2.publish(statePub, getTransformStamped(robotHandle,'P3-DX',-1,'world'))
    end
end 

