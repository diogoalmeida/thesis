function cd_c_comments(fin, fout)

% Replace the C++ style comments with ANSI C style comments
% // text
% will be replaced by
% /* text */

s = fileread(fin);
a = regexprep(s, '//([^\r\n]*)\r\n', '/*$1 */\r\n');
f = fopen(fout,'w');
fwrite(f, a, 'char*1');
fclose(f);
