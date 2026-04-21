function ca_code_matrix = generate_ca_code(prn, rows)
    % Generate C/A code for a given PRN number and create a matrix with specified rows
    % Inputs:
    %   prn - PRN number of the satellite (integer, 1-32)
    %   rows - Number of rows for the output matrix
    % Output:
    %   ca_code_matrix - Matrix with C/A code repeated in rows x 1023

    % Define the G2 tap configuration for each PRN
    g2_taps = [
        2, 6;   % PRN 1
        3, 7;   % PRN 2
        4, 8;   % PRN 3
        5, 9;   % PRN 4
        1, 9;   % PRN 5
        2, 10;  % PRN 6
        1, 8;   % PRN 7
        2, 9;   % PRN 8
        3, 10;  % PRN 9
        2, 3;   % PRN 10
        3, 4;   % PRN 11
        5, 6;   % PRN 12
        6, 7;   % PRN 13
        7, 8;   % PRN 14
        8, 9;   % PRN 15
        9, 10;  % PRN 16
        1, 4;   % PRN 17
        2, 5;   % PRN 18
        3, 6;   % PRN 19
        4, 7;   % PRN 20
        5, 8;   % PRN 21
        6, 9;   % PRN 22
        1, 3;   % PRN 23
        4, 6;   % PRN 24
        5, 7;   % PRN 25
        6, 8;   % PRN 26
        7, 9;   % PRN 27
        8, 10;  % PRN 28
        1, 6;   % PRN 29
        2, 7;   % PRN 30
        3, 8;   % PRN 31
        4, 9;   % PRN 32
    ];

    % Initialize the G1 and G2 shift registers
    g1 = ones(1, 10); % G1 shift register (10 bits)
    g2 = ones(1, 10); % G2 shift register (10 bits)
    
    % Get the tap positions for the selected PRN
    g2_shift1 = g2_taps(prn, 1);
    g2_shift2 = g2_taps(prn, 2);
    
    ca_code = zeros(1, 1023); % Initialize the C/A code sequence
    
    for i = 1:1023
        ca_code(i) = xor(g1(10), xor(g2(g2_shift1), g2(g2_shift2))); % Generate the C/A code bit
        
        % Update G1 register
        g1_feedback = xor(g1(3), g1(10));
        g1 = [g1_feedback g1(1:9)];
        
        % Update G2 register
        g2_feedback = xor(xor(xor(xor(xor(g2(2), g2(3)), g2(6)), g2(8)), g2(9)), g2(10));
        g2 = [g2_feedback g2(1:9)];
    end
    
    % Generate the output matrix
     ca_code_matrix1 = repelem(ca_code,16);
    ca_code_matrix = repmat(ca_code_matrix1, rows, 1);
    ca_code_matrix(ca_code_matrix == 0) = -1;
end
