function newY = verticalMove(rWheelV,lWheelV,theta)
    enFrequency = 10; %(Hz)
    enTime = 1/enFrequency; %(s)

    newY = ((rWheelV/2)+(lWheelV/2)) * sin(theta) * enTime;

end