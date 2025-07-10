clf
r = [428.6666667,279,127.5,57,45.33333333,37,11.66666667,3.71,1.193333333,0.5643333333];

logr = log10(r);

force = [0.11368,0.21168,0.30968, 0.40768, 0.50568, 0.60368, 1.09368, 2.07368, 5.01368, 9.91368];

logforce = log10(force);

error = [19.13983629, 55.42562584, 91.51639197, 8.185352772, 10.69267662, 3, 2.081665999, 0.7907591289, 0.1833939294, 0.01913983629];

logerror = 0.434.*(error./r);

x = linspace(min(force),max(force),100);
y = 16.1*x.^(-1.59);
errorbar(force, r, error, 'bo', 'DisplayName','Actual Data','MarkerSize',10);
hold on
grid on
loglog(x,y, 'r','DisplayName','y = 16.1x^{(-1.59)}, R^2 = 0.99');
xscale('log');
yscale('log');
xlim([0,11]);
xlabel('Log Applied Force [N]');
ylabel('Log Resistance [kOhms]');
title('Log-Log Plot of Resistance vs Applied Force');
legend('Location','best');
set(gca, 'fontsize', 14);

