
function encoder_steps = radians_to_encoder_steps(angle_in_radian, joint_transmission)
    ENCODER_PULSES_PER_REVOLUTION = 2000;
    radians_per_step = (2*pi)/(joint_transmission * ENCODER_PULSES_PER_REVOLUTION);
    encoder_steps = angle_in_radian/radians_per_step;
end