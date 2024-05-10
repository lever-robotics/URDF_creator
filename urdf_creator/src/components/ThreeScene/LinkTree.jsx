import React from 'react';

// RecursiveTreeView Component
export const LinkTree = ({ tree }) => {
    console.log(tree)
    // Function to render each node and its children
    const renderNode = (node) => {
        if (!node) {
            return null;
        }


        // Display the current node's data and recursively render its children
        return (
        <div style={{ marginLeft: '20px' }}>
            {node.userData && (<div>Name: {node.userData.shape}</div>)}
            {node.children && node.children.length > 0 && (
            <div>
                {node.children.filter((child) => child.type === "Mesh").map((child) => renderNode(child))}
            </div>
            )}
        </div>
        );
    };

    return (
        <div>
        <div className='filled-box'>
            <h3>Tree View</h3>
            { tree && renderNode(tree.object) }
        </div>
        </div>
    );
};

