#ifndef PIDControl_H
#define PIDControl_H


template <class T> class PIDControl{

        public:
            float _p, _i, _d;
            T target, currentFeedback, lastFeedback, output, error, lastError;
            T integralCumulation, maxCumulation, cycleDerivative, maxDerivative;
            bool enabled;

            bool inputBounded, outputBounded;
            T inputLowerBound, inputUpperBound, outputLowerBound, outputUpperBound;

            bool feedbackWrapped;
            T feedbackWrapLowerBound, feedbackWrapUpperBound;

            bool timeFunctionRegistered;
            uint64_t currentTime, lastTime;
    
        public:
            //Constructor y funciones
            PIDControl(float, float, float);
            void tick(T sensor, T mando);

            //Setters
            
            void setTarget(T t);
            void setEnabled(bool e);
            void setMaxIntegralCumulation(T max);
            void setInputBounded(bool bounded);
            void setInputBounds(T lower, T upper);
            void setOutputBounded(bool bounded);
            void setOutputBounds(T lower, T upper);
            void setFeedbackWrapped(bool wrapped);
            void setFeedbackWrapBounds(T lower, T upper);
            void setPID(T p, T i, T d);
            void setP(T p);
            void setI(T i);
            void setD(T d);
            void setCurrentFeedback(T pidSource);
            void setOutput(T pidOutput);
            void setCurrentTime(long time);
            void setLastTime(long time);
            

            //Getters
            T getTarget();
            T getOutput();
            T getCurrentFeedback();
            T getError();
            T getProporcionalComponent();
            T getIntegralComponent();
            T getDerivadaComponent();
            T getMaxIntegralCumulation();
            T getIntegralCumulation();
            T getInputLowerBound();
            T getInputUpperBound();
            T getOutputLowerBound();
            T getOutputUpperBound();
            T getFeedbackWrapLowerBound();
            T getFeedbackWrapUpperBound();
            T getP();
            T getI();
            T getD();

            //Booleans
            bool isEnabled();
            bool isInputBounded();
            bool isOutputBounded();
            bool isFeedbackWrapped();
    

};

#endif