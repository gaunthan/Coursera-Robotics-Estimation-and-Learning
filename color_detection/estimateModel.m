function [mu, sigma] = estimateModel(Samples, ndim)

data = double(Samples(:, 1:ndim));
N = size(data, 1);

% mu = mean(data, 1);
mu = 1/N * sum(data, 1);

% sigma = cov(data);
sigma = zeros(size(mu, 2));
devi = data - mu;
for i=1:N
    sigma = sigma + devi(i, :)' * devi(i, :);
end
sigma = 1 / N * sigma;

% Plot the model
if ndim == 2
    figure,
    scatter(data(:, 1), data(:, 2), '.');
    title('Pixel Color Distribubtion');
    xlabel('Red');
    ylabel('Green');
    hold on

    x1 = min(min(data)):max(max(data));
    x2 = x1;
    [X1, X2] = meshgrid(x1, x2);
    F = mvnpdf([X1(:), X2(:)], mu, sigma);
    F = reshape(F, length(x2), length(x1));
    contour(x1, x2, F, [.0001 .001 .01 .05:.1:.95 .99 .999 .9999]);
    plot(mu(1), mu(2), 'r*');
end

thre = 0.001;

save parameters.mat mu sigma thre

end