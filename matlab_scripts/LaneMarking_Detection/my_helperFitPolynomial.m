function [P,score] = my_helperFitPolynomial(pts,degree,resolution)
P = fitPolynomialRANSAC(pts,degree,resolution);
ptsSquare = (polyval(P,pts(:,1)) - pts(:,2)).^2;
score =  sqrt(mean(ptsSquare));
end