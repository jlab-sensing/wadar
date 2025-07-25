from graphviz import Digraph

dot = Digraph(comment='Radar Data Processing Pipeline', format='pdf')

# Set graph attributes for a cleaner, publication-ready look
dot.attr(rankdir='LR', size='8,5', fontsize='14', fontname='Times-Roman')
dot.attr('node', shape='box', style='filled', color='lightgrey', fontname='Times-Roman', fontsize='12')

# Nodes
dot.node('A', 'Raw Radar Data\n(Fast Time × Slow Time)', shape='box', style='filled,bold', color='lightblue')

dot.node('B1', 'Handcrafted Features', shape='box')
dot.node('B2', 'Classical Dimensionality Reduction', shape='box')
dot.node('B3', 'Deep Learning Feature Extraction', shape='box')

dot.node('C1', 'Classical Regression Models', shape='box')
dot.node('C2', 'Deep Learning Regression Models', shape='box')
dot.node('C3', 'End-to-End Deep Learning Models', shape='box')

dot.node('D1', 'Bulk Density\n(g/cm³)', shape='box', style='filled,bold', color='lightyellow')
dot.node('E1', 'Compaction Classification', shape='box', style='filled,bold', color='lightgreen')

# Edges
dot.edge('A', 'B1')
dot.edge('A', 'B2')
dot.edge('A', 'B3')

dot.edge('B1', 'C1')
dot.edge('B2', 'C1')
dot.edge('B3', 'C1')
dot.edge('B1', 'C2')
dot.edge('B2', 'C2')
dot.edge('B3', 'C2')
dot.edge('A', 'C3')

dot.edge('C1', 'D1')
dot.edge('C2', 'D1')
dot.edge('C3', 'D1')
dot.edge('D1', 'E1')

# Subgraphs to make end-to-end sit with the right cluster
with dot.subgraph(name='cluster_feature') as c:
    c.attr(style='dashed', label='Feature Extraction')
    c.node('B1')
    c.node('B2')
    c.node('B3')

with dot.subgraph(name='cluster_model') as c:
    c.attr(style='dashed', label='Regression')
    c.node('C1')
    c.node('C2')
    c.node('C3')

dot.render('flowchart', cleanup=True)
