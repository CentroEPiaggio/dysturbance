function type = find_type(data)

[n1,n2] = size(data);

if n1 == 1 && n2 == 1
    type = "scalar";
elseif (n1 == 1 && n2 ~= 1) || (n2 == 1 && n1 ~= 1)
    type = "vector";
elseif (n2 > 1 && n1 > 1)
    type = "matrix";
else 
    error("Error: data for data type evaluation is empty!! \n");
end

end