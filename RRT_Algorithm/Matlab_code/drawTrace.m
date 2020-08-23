function f=drawTrace(la, lb, lc, ld, le, lf,COLOR)
line([la(1) lb(1)],[la(2) lb(2)],'color',COLOR);
line([lb(1) lc(1)],[lb(2) lc(2)],'color',COLOR);
line([lc(1) ld(1)],[lc(2) ld(2)],'color',COLOR);
line([le(1) lf(1)],[le(2) lf(2)],'color',COLOR);
end