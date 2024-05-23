numMarkers = 1;
markerSize = 50; % in pixels
markerFamily = "DICT_4X4_250";
ids = 1:numMarkers;

imgs = generateArucoMarker(markerFamily,ids,markerSize);

tiledlayout(3,4,TileSpacing="compact")
for i = 1:numMarkers
    nexttile
    imshow(imgs(:,:,i))
    title("ID = " + ids(i))
end

for i = 1:numMarkers
    filename = markerFamily + "_ID" + ids(i) + ".png";
    imwrite(imgs(:,:,i), filename)
end