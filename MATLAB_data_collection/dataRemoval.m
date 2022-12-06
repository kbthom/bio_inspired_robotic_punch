function [c, d] = dataRemoval(a,b)
    [max_val, index] = max(a);
    c = a([1:index-1, index+1:end])
    d = b([1:index-1, index+1:end])
end