import React from 'react';

// RecursiveTreeView Component
export const LinkTree = ({ tree, select }) => {
    console.log(tree)
    // Function to render each node and its children
    const renderNode = (node) => {
        if (!node) {
            return null;
        }


        // Display the current node's data and recursively render its children
        return (
            <div style={{ marginLeft: '20px' }}>
                {node.userData && (<button className='tree-item' onClick={() => {
                    select(node);
                    console.log("clicked")
                }}>{node.userData.name}</button>)}
                {node.children && node.children.length > 0 && (
                    <div>
                        {node.children.filter((child) => child.type === "Mesh").map((child) => renderNode(child))}
                    </div>
                )}
            </div>
        );
    };

    let node = null;
    if (tree) {
        if (tree.children) {
            const children = tree.children.filter((child) => child.type === "Mesh")
            if (children.length > 0) {
                node = children[0]
            }
        }
    }

    return (
        <div className='object-tree'>
            Object Tree
            <div className='scroll-box'>
                {node && renderNode(node)}
            </div>

        </div>
    );
};

