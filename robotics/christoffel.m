function C = christoffelMatrix(christoffel_parameters,q_dot)
   
    syms C

    for k=1:length(q_dot)
        for j=1:length(q_dot)
            C(k,j) = 0; % Initially zero
            for i=1:length(q_dot)
                C(k,j) = C(k,j) + christoffel_parameters(i,j,k)*q_dot(i);
            end
        end
    end  

end