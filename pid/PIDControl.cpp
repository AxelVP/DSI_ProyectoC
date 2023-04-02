#include <iostream>
#include <chrono>
#include "PIDControl.h"

//sustituir T por float en el main

using namespace std::chrono;

template <class T> PIDControl<T>::PIDControl(float p, float i, float d){
    _p = p;
    _i = i;
    _d = d;

    target = 0;             //Posición deseada del dron
    currentFeedback = 0;    //Posición actual del dron
    output = 0;             //Salida del PID
    enabled = true;         //Habilita la actuacion del tick, que actualiza el output
    lastFeedback = 0;       //Anterior posicion registrada
    error = 0;              //Error 
    lastError = 0;          //Anterior error registrado
    currentTime = 0L;       //Marca temporal actual el sistema
    lastTime = 0L;          //Anterior marca temporal  registrada
    integralCumulation = 0; //Total del valor acumulado que lleva la integral (la suma de todos los cycleIntegral)
    maxCumulation = 30;  //Maximo que puede llegar a pillar la integral -> Si se pasa iguala la integral a dicho valor
    maxDerivative = 30;
    cycleDerivative = 0;    //Valor de la derivada

    inputBounded = false;   //A true si queremos limitar la entrada de los sensores
    inputLowerBound = 0;
    inputUpperBound = 0;    
    outputBounded = false;  //A true si queremos limitar la salida del PID
    outputLowerBound = 0;   
    outputUpperBound = 0;   
    
    feedbackWrapped = true;  //Que el PID automaticamente se autocorrija. Por ejemplo, si output da +300 grados, que se corrija a -60 grados
    feedbackWrapLowerBound = 0;
    feedbackWrapUpperBound = 0;

    timeFunctionRegistered = true;  //A true si queremos registrar el tiempo y con ello calcular la derivada e integral
}

template <class T> void PIDControl<T>::tick(T sensor, T mando){
    if(enabled) {
        currentFeedback = sensor;
        target = mando;

        if(inputBounded){                                                                     //si se limita la entrada
            if(currentFeedback > inputUpperBound) currentFeedback = inputUpperBound;
            if(currentFeedback < inputLowerBound) currentFeedback = inputLowerBound;
        }

        if(feedbackWrapped){                                           //si se desea corregir el error al minimo movimiento (pasar de +300 grados a -60)  
            float regErr = target - currentFeedback;
            float altErr1 = (target - feedbackWrapLowerBound) + (feedbackWrapUpperBound - currentFeedback);       //Poner estos bounds a 0 y 360 con el setter
            float altErr2 = (feedbackWrapUpperBound - target) + (currentFeedback - feedbackWrapLowerBound);

            //Calculate the absolute values of each error.
            float regErrAbs = (regErr >= 0) ? regErr : -regErr;
            float altErr1Abs = (altErr1 >= 0) ? altErr1 : -altErr1;
            float altErr2Abs = (altErr2 >= 0) ? altErr2 : -altErr2;

            //Use the error with the smallest absolute value
            if(regErrAbs <= altErr1Abs && regErr <= altErr2Abs) //If reguErrAbs is smallest
            {
                error = regErr;
            }
            else if(altErr1Abs < regErrAbs && altErr1Abs < altErr2Abs) //If altErr1Abs is smallest
            {
                error = altErr1Abs;
            }
            else if(altErr2Abs < regErrAbs && altErr2Abs < altErr1Abs) //If altErr2Abs is smallest
            {
                error = altErr2Abs;
            }

        }else {
            error = target - currentFeedback;
        }

        if(timeFunctionRegistered) {  
            currentTime = duration_cast< microseconds >(system_clock::now().time_since_epoch()).count();
            long deltaTime = currentTime - lastTime;
            float dt = deltaTime / 1000000.0;
            //std::cout << "dt: " << dt << std::endl; 
            float cycleIntegral = ((lastError + error) / 2.0) * dt;            
            integralCumulation += cycleIntegral;                                //calcula la integral
            cycleDerivative = (error - lastError) / dt;                  //calcula la derivada
            lastTime = currentTime;
        } else {
            integralCumulation += error;
            cycleDerivative = (error - lastError);
        }

        if(integralCumulation > maxCumulation) integralCumulation = 0;
        if(integralCumulation < -maxCumulation) integralCumulation = 0;  

        if(cycleDerivative > maxDerivative) cycleDerivative = 0;
        if(cycleDerivative < -maxDerivative) cycleDerivative = 0;  

        //Calculamos la salida del PID
        /*std::cout << "ERROR " << error << ", Kp " << _p << ", Integral " << integralCumulation << ", Ki " << _i << ", Derivada " << cycleDerivative << ", Kd " << _d << std::endl;*/
        
        std::cout << "ERROR " << error << " I " << integralCumulation << " D " << cycleDerivative << std::endl;
        output = (float) ((error * _p) + (integralCumulation * _i) + (cycleDerivative * _d));

        //Guardamos el estado del Feedback y del error para la siguiente interación
        lastFeedback = currentFeedback;
        lastError = error;

        //If por si queremos limitar la salida del PID
        if(outputBounded) {
            if(output > outputUpperBound) output = outputUpperBound;
            if(output < outputLowerBound) output = outputLowerBound;
        }

        setOutput(output);
    }
}

        
/***********************SETTERS*********************/
template <class T> void PIDControl<T>::setTarget(T t) {
    target = t;
}

template <class T> void PIDControl<T>::setEnabled(bool e) {
    //If the PIDControl was enabled and is being disabled.
    if(!e && enabled)
    {
        output = 0;
        integralCumulation = 0;
    }
    enabled = e;
}

template <class T> void PIDControl<T>::setMaxIntegralCumulation(T max) {
    //If the new max value is less than 0, invert to make positive.
    if(max < 0)
    {
        max = -max;
    }

    //If the new max is not more than 1 then the cumulation is useless.
    if(max > 1)
    {
        maxCumulation = max;
    }
}

template <class T> void PIDControl<T>::setInputBounded(bool bounded) {
    inputBounded = bounded;
}

template <class T> void PIDControl<T>::setInputBounds(T lower, T upper) {
    if(upper > lower)
    {
        inputBounded = true;
        inputUpperBound = upper;
        inputLowerBound = lower;
    }
}

template <class T> void PIDControl<T>::setOutputBounded(bool bounded) {
    outputBounded = bounded;
}

template <class T> void PIDControl<T>::setOutputBounds(T lower, T upper) {
    if(upper > lower){
        outputBounded = true;
        outputLowerBound = lower;
        outputUpperBound = upper;
    }
}

template <class T> void PIDControl<T>::setFeedbackWrapped(bool wrapped) {
    feedbackWrapped = wrapped;
}

template <class T> void PIDControl<T>::setFeedbackWrapBounds(T lower, T upper) {
    //Make sure no value outside this circular range is ever input.
    setInputBounds(lower, upper);

    feedbackWrapped = true;
    feedbackWrapLowerBound = lower;
    feedbackWrapUpperBound = upper;
}

template <class T> void PIDControl<T>::setPID(T p, T i, T d) {
    _p = p;
    _i = i;
    _d = d;
}

template <class T> void PIDControl<T>::setP(T p) {
    _p = p;
}

template <class T> void PIDControl<T>::setI(T i) {
    _i = i;
}

template <class T> void PIDControl<T>::setD(T d) {
    _d = d;
}

template <class T> void PIDControl<T>::setCurrentFeedback(T pidSource) {
    currentFeedback = pidSource;
}

template <class T> void PIDControl<T>::setOutput(T pidOutput) {
    output = pidOutput;
}

template <class T> void PIDControl<T>::setCurrentTime(long time){
    currentTime = time;
}

template <class T> void PIDControl<T>::setLastTime(long time){
    lastTime = time;
}


/***********************GETTERS*********************/
template <class T> T PIDControl<T>::getTarget() {
    return target;
}

template <class T> T PIDControl<T>::getOutput(){
    return output;
}

template <class T> T PIDControl<T>::getCurrentFeedback() {
    return currentFeedback;
}

template <class T> T PIDControl<T>::getError() {
    return error;
}

template <class T> T PIDControl<T>::getProporcionalComponent() {
    return (T) (error * _p);
}

template <class T> T PIDControl<T>::getIntegralComponent() {
    return (T) (integralCumulation * _i);
}

template <class T> T PIDControl<T>::getDerivadaComponent() {
    return (T) (cycleDerivative * _d);
}

template <class T> T PIDControl<T>::getMaxIntegralCumulation() {
    return maxCumulation;
}

template <class T> T PIDControl<T>::getIntegralCumulation() {
    return integralCumulation;
}

template <class T> T PIDControl<T>:: getInputLowerBound() {
    return inputLowerBound;
}

template <class T> T PIDControl<T>::getInputUpperBound() {
    return inputUpperBound;
}

template <class T> T PIDControl<T>::getOutputLowerBound() {
    return outputLowerBound;
}

template <class T> T PIDControl<T>::getOutputUpperBound() {
    return outputUpperBound;
}

template <class T> T PIDControl<T>::getFeedbackWrapLowerBound() {
    return feedbackWrapLowerBound;
}

template <class T> T PIDControl<T>::getFeedbackWrapUpperBound() {
    return feedbackWrapUpperBound;
}

template <class T> T PIDControl<T>::getP() {
    return _p;
}

template <class T> T PIDControl<T>::getI() {
    return _i;
}

template <class T> T PIDControl<T>::getD() {
    return _d;
}


/***********************BOOLEANS*********************/
template <class T> bool PIDControl<T>::isEnabled() {
    return enabled;
}

template <class T> bool PIDControl<T>::isInputBounded() {
    return inputBounded;
}

template <class T> bool PIDControl<T>::isOutputBounded() {
    return outputBounded;
}

template <class T> bool PIDControl<T>::isFeedbackWrapped() {
    return feedbackWrapped;
}

template class PIDControl<int>;
template class PIDControl<long>;
template class PIDControl<float>;
template class PIDControl<double>;

 




