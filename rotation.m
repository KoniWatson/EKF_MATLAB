function theta = rotation(rWheelV,lWheelV)
    wheelBase = 120; %(mm)
    enFrequency = 10; %(Hz)
    enTime = 1/enFrequency; %(s)
    
    theta = ((1/wheelBase)*(rWheelV-lWheelV) * enTime);
end