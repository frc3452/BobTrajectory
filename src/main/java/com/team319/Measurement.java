package com.team319;

public class Measurement {
    
    private final double inches;

    public Measurement(double value, boolean feet)
    {
        value = Math.abs(value);
        this.inches = value * (feet ? 12 : 1);
    }

    public Measurement(double value)
    {
        this(value, false);
    }

    public double f()
    {
        return this.inches/12;
    }

    public double in()
    {
        return this.inches;
    }

    public double f_2()
    {
        return f()/2;
    }

    public double in_2()
    {
        return in()/2;
    }

}