function beaconPoints = trilateration(x,y,z,noise,b1,b2,b3)
    U = b2(1) - b1(1);
    Vx = b3(1) - b1(1);
    Vy = b3(2) - b1(2);
    V = Vx^2 + Vy^2;

    r1 = x^2 + y^2 + z^2;
    r2 = (x - U)^2 + y^2 + z^2;
    r3 = (x - Vx)^2 + (y - Vy)^2 + z^2;
    
    xBot = ((sqrt(r1)+(sqrt(noise)*randn(1)))^2 - (sqrt(r2)+(sqrt(noise)*randn(1)))^2 + U^2) / (2 * U);
    yBot = ((sqrt(r1)+(sqrt(noise)*randn(1)))^2 - (sqrt(r3)+(sqrt(noise)*randn(1)))^2 + V - (2 * Vx * x)) / (2 * Vy);
    
    beaconPoints = [xBot, yBot];
end





