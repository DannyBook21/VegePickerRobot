clear;
clc;

a = arduino();

buttonPin = 'D2';

configurePin(a, buttonPin, 'Pullup');

prevButtonState = readDigitalPin(a, buttonPin);

while true
    buttonState = readDigitalPin(a, buttonPin);
    
    if buttonState == 0 && prevButtonState == 1
        disp('Button Pressed!');
        while readDigitalPin(a, buttonPin) == 0
        end
    end
    
    prevButtonState = buttonState;
end
