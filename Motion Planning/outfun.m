function stop = outfun(x,optim_option,state,~)
stop = false;

switch state
    case 'init'
        %hold on
    case 'iter'
        filename = sprintf(strcat('fmincon_iter',num2str(optim_option.iteration)));
        fval = optim_option.fval;
        constrviolation = optim_option.constrviolation;
        %curdir = pwd;
        svpath = strcat(pwd,'\Solver Results\');
        if constrviolation < 5
            if constrviolation < 1
                filename = sprintf(strcat(filename,'_AWESOME'));
            end
            save([svpath filename],'x','fval','constrviolation');
        end
        format short
        %disp(x(1:5).');
    case 'done'
        %hold off
    otherwise
end
end