#ifndef stepperMotor_h
#define stepperMotor_h

class steppermotor {
    public:
        //constructor
        steppermotor(int microstep,int directionPinA,int pulsePinA, int directionPinB, int pulsePinB);

        //function
        void stepperSetspeed(int desired_speed);
        float stepperMove(int motorside, float desired_step);

        volatile bool  DirForward;
    private:

        //variables
        unsigned long stepperSpeed;
        int microstep;
        int pulsePinL, pulsePinR;
        int directionPinL, directionPinR;
       
};      

#endif
