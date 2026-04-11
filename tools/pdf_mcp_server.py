import os
import mcp.types as types
from mcp.server import Server, NotificationOptions
from mcp.server.models import InitializationOptions
import mcp.server.stdio
import pymupdf4llm

server = Server("pdf-reader-server")

@server.list_tools()
async def handle_list_tools() -> list[types.Tool]:
    """
    List available tools.
    """
    return [
        types.Tool(
            name="read_pdf_with_math",
            description="Reads a PDF file and extracts its content as Markdown, preserving LaTeX math formulas. Returns the Markdown content.",
            inputSchema={
                "type": "object",
                "properties": {
                    "pdf_path": {
                        "type": "string",
                        "description": "The absolute path to the PDF file."
                    }
                },
                "required": ["pdf_path"],
            },
        )
    ]

@server.call_tool()
async def handle_call_tool(
    name: str, arguments: dict | None
) -> list[types.TextContent | types.ImageContent | types.EmbeddedResource]:
    """
    Handle tool execution requests.
    """
    if name != "read_pdf_with_math":
        raise ValueError(f"Unknown tool: {name}")

    if not arguments or "pdf_path" not in arguments:
        raise ValueError("Missing pdf_path argument")

    pdf_path = arguments["pdf_path"]
    
    try:
        # Pymupdf4llm turns the pdf into a markdown string directly. 
        md_text = pymupdf4llm.to_markdown(doc=pdf_path)
        
        return [
            types.TextContent(
                type="text",
                text=f"PDF parsed successfully. Math formulas are rendered via Markdown.\n\n### Document Content ###\n\n{md_text}"
            )
        ]
    except Exception as e:
        return [
            types.TextContent(
                type="text",
                text=f"Failed to read PDF. Error: {str(e)}"
            )
        ]

async def main():
    async with mcp.server.stdio.stdio_server() as (read_stream, write_stream):
        await server.run(
            read_stream,
            write_stream,
            InitializationOptions(
                server_name="pdf-reader",
                server_version="0.1.0",
                capabilities=server.get_capabilities(
                    notification_options=NotificationOptions(),
                    experimental_capabilities={},
                ),
            ),
        )

if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
