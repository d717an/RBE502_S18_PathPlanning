% Path generator
function yd = path_generator(amplitude, freq, t)
    yd = amplitude * sin(freq * t);
end