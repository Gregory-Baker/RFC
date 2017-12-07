function transform = DHTransform(a_n, alpha_n, d_n, theta_n)

    c_t = cos(theta_n);
    s_t = sin(theta_n);
   
    c_a = cos(alpha_n);
    s_a = sin(alpha_n);
    
 
    transform = [c_t, -c_a*s_t,  s_a*s_t, a_n*c_t;
                 s_t,  c_a*c_t, -s_a*c_t, a_n*s_t;
                   0,      s_a,      c_a,     d_n;
                   0,        0,        0,       1];
               
end