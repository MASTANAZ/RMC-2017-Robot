%look_ahead

function updated_grid = look_ahead(true_grid, estiRover)
true_grid
newX = estiRover.pos(2) + cos(estiRover.pos(3) * pi / 180);
newY = estiRover.pos(1) + cos(estiRover.pos(3) * pi / 180);

updated_grid = true_grid(round(estiRover.pos(2)+newX), round(estiRover.pos(1)+newY))

end