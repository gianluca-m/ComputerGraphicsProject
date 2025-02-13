obsFrequencies = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 50415, 67, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 314, 390238, 5096, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2934, 467005, 18945, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1411, 57454, 5081, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 116, 25, 0, 0, 0, 0, 0, 0, 0 ];
expFrequencies = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.38435e-21, 1.56638, 50124.3, 76.2399, 1.74459e-16, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 3.72008e-30, 8.0596e-10, 301.798, 391146, 5028.03, 4.55452e-07, 6.6933e-26, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 1.03056e-31, 3.39359e-16, 0.000159113, 2891.51, 466974, 18833.7, 0.00721162, 8.62198e-14, 1.09617e-28, 0, 0, 0, 0; 0, 0, 0, 0, 0, 4.98355e-37, 3.60358e-27, 1.04754e-17, 1.94659e-09, 0.0145155, 1385.92, 57085.1, 5049.47, 0.171486, 4.29406e-08, 4.08289e-16, 2.07045e-25, 2.86644e-35, 0, 0; 1.56504e-13, 1.56504e-13, 1.56504e-13, 1.56504e-13, 1.56504e-13, 1.56505e-13, 1.57635e-13, 9.94734e-12, 9.15257e-08, 0.00537163, 9.78879, 118.624, 21.9659, 0.0292074, 6.83307e-07, 4.65174e-11, 1.62545e-13, 1.56505e-13, 1.56505e-13, 1.56505e-13 ];
colormap(jet);
clf; subplot(2,1,1);
imagesc(obsFrequencies);
title('Observed frequencies');
axis equal;
subplot(2,1,2);
imagesc(expFrequencies);
axis equal;
title('Expected frequencies');
