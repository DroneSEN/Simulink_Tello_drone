# Object detection

This document is dedicated to the Yolo Object Detection system.

\documentclass{standalone}
\usepackage{tikz}
\usetikzlibrary{shapes.geometric, arrows, positioning, calc}

\tikzstyle{block} = [rectangle, draw, fill=blue!20, 
    text width=12em, text centered, rounded corners, minimum height=4em]
\tikzstyle{line} = [draw, -latex']
\tikzstyle{database} = [cylinder, shape border rotate=90, draw, minimum height=4em, minimum width=2em, text width=10em, text centered, fill=blue!20]

\begin{document}

\begin{tikzpicture}[node distance = 2cm, auto]
    % Nodes
    \node [database] (known) {Base de données d'objets connus \\ (x, y, z, h, w, p)};
    \node [database, below of=known] (estimated) {Base de données d'objets de grande taille estimée \\ (h, w, p)};
    \node [block, right of=known, node distance=5cm] (algo) {Algorithme};
    \node [block, right of=algo, node distance=5cm] (drone) {Repositionnement du drone};
    \node [block, below of=drone] (map) {Carte sémantique};
    \node [block, below of=estimated, node distance=5cm] (yolo) {YOLO (You Only Look Once) reconnaissance d'objets \\ (x, y, h, w)};
    \node [block, below of=algo, node distance=5cm] (camera) {Paramètre intrinsèque Caméra};
    \node [below of=camera, node distance=2cm] (matrix) {
        $K = \begin{bmatrix}
        f_x & 0 & c_x \\
        0 & f_y & c_y \\
        0 & 0 & 1
        \end{bmatrix}$
    };

    % Lines
    \path [line] (known) -- (algo);
    \path [line] (estimated) -- (algo);
    \path [line] (yolo) -- (algo);
    \path [line] (algo) -- (drone);
    \path [line] (algo) -- (map);
    \path [line] (camera) -- (algo);
\end{tikzpicture}

\end{document}
