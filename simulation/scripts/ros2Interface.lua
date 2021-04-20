function sysCall_init() 
    
    robotHandle=sim.getObjectHandle(sim.handle_self)
    leftMotor=sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    rightMotor=sim.getObjectHandle("Pioneer_p3dx_rightMotor")

    if simROS2 then
        
        sim.addLog(sim.verbosity_scriptinfos, "ROS2 interface was found.")
        
        leftMotorTopicName='leftMotorSpeed'
        rightMotorTopicName='rightMotorSpeed'
        
        simTimeTopicName='simTime'
        
        simTimePub=simROS2.createPublisher('/'..simTimeTopicName, 'std_msgs/msg/Float32')
        
        leftMotorSub=simROS2.createSubscription('/'..leftMotorTopicName, 'std_msgs/msg/Float32', 'setLeftMotorSpeed')
        rightMotorSub=simROS2.createSubscription('/'..rightMotorTopicName, 'std_msgs/msg/Float32', 'setRightMotorSpeed')
        
        result=sim.launchExecutable('pioneer',leftMotorTopicName.." "..rightMotorTopicName.." "..simTimeTopicName,0)
    else
       
       sim.addLog(sim.verbosity_scriptinfos, "ROS2 Interface was not found. Cannot run. ")
    
    end

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
    end
end 

