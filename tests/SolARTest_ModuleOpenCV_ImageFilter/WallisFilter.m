imgA = imread("D:\Dev\SolAR\modules\SolARModuleOpenCV\tests\data\notredame1.jpg");
imgAd = im2uint8(CBM3D(im2double(imgA), 0.11)); 
imshow(imgAd);